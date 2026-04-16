import time
import math
import rclpy
import struct
import numpy as np
from rclpy.node import Node
from library.robot_big import Robot
from library.communication import Communication
from library.utils import angle_difference, clamp
from path_planner import AStarPlanner

import json
import heapq
import os

def is_point_in_poly(x, y, poly):
    n = len(poly)
    inside = False
    if n == 0: return False
    p1x, p1y = poly[0]['x'], poly[0]['y']
    for i in range(n + 1):
        p2x, p2y = poly[i % n]['x'], poly[i % n]['y']
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def load_map(file_path="arena_map.json", max_y=5.0):
    if not os.path.exists(file_path):
        return 6.0, max_y, []
        
    with open(file_path, "r") as f:
        data = json.load(f)
    
    filtered_obstacles = []
    for obs in data.get("obstacles", []):
        if any(p['y'] <= max_y for p in obs):
            filtered_obstacles.append(obs)
            
    return data.get("arena_size", {}).get("width", 6.0), max_y, filtered_obstacles


class GlobalNavigator(Node):
    def __init__(self, robot: Robot, comm: Communication):
        super().__init__("global_navigator")
        self.robot = robot
        self.comm = comm
        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0
        self.visible = False
        self.last_log_time = time.time()
        
        # MIRROR CONFIGURATION
        self.mirror_x = True 
        self.arena_width = 6.0
        
        # Register the location callback
        self.comm.register_callback_location(self._location_cb)
        
        # Load map and initialize planner (limited to y <= 5.0)
        width, height, obstacles = load_map("arena_map.json", max_y=5.0)
        # Higher robot radius (0.45m) for the master robot safety
        self.planner = AStarPlanner(width, 10.0, obstacles, robot_radius=0.45)
        
        self.get_logger().info("Mecanum Holonomic Navigator (Static Map) initialized. Waiting for camera...")

    def _location_cb(self, x, y, angle, visible, last_seen):
        if self.mirror_x:
            self.pos_x = self.arena_width - x
            # Mirrored heading: pi - angle
            self.heading = math.pi - angle
        else:
            self.pos_x = x
            self.heading = angle
        self.pos_y = y
        self.visible = visible

    def wait_for_location(self):
        """Block until the overhead camera sees the robot."""
        while rclpy.ok() and not self.visible:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Location locked: ({self.pos_x:.2f}, {self.pos_y:.2f})")

    def navigate_to(self, goal_x, goal_y, timeout=180.0):
        if not self.visible:
            self.get_logger().error("Cannot plan: Robot not visible!")
            return False

        start_time = time.time()
        start_pos = (self.pos_x, self.pos_y)
        goal_pos = (goal_x, goal_y)
        
        self.get_logger().info(f"Planning path: {start_pos} -> {goal_pos}")
        path = self.planner.plan(start_pos, goal_pos)
        
        if not path:
            self.get_logger().error("No path found to goal!")
            return False

        self.get_logger().info(f"Path found with {len(path)} waypoints. Executing...")
        
        current_wp_idx = 0
        
        while rclpy.ok() and current_wp_idx < len(path):
            rclpy.spin_once(self, timeout_sec=0)
            
            now = time.time()
            
            # 0. TIMEOUT CHECK
            if now - start_time > timeout:
                self.get_logger().error(f"Navigation timed out after {timeout} seconds!")
                self.robot.drive(0.0, 0.0, 0.0)
                return False

            if not self.visible:
                if now - self.last_log_time > 1.0:
                    self.get_logger().warn("[!] Camera Lost - Stopping and Waiting...")
                    self.last_log_time = now
                self.robot.drive(0.0, 0.0, 0.0)
                time.sleep(0.1)
                continue

            # 1. PATH PRUNING: Look ahead to skip waypoints if we are already close to a later one
            for look_ahead in range(len(path)-1, current_wp_idx, -1):
                lx, ly = path[look_ahead]
                if math.hypot(lx - self.pos_x, ly - self.pos_y) < 0.25:
                    current_wp_idx = look_ahead
                    break

            wx, wy = path[current_wp_idx]
            dx = wx - self.pos_x
            dy = wy - self.pos_y
            distance = math.hypot(dx, dy)
            
            # If reached current waypoint, move to next
            if distance < 0.2:
                current_wp_idx += 1
                continue

            # 2. HOLONOMIC MOVEMENT (Mecanum)
            global_target_angle = math.atan2(dy, dx)
            relative_angle = angle_difference(global_target_angle, -self.heading)
            
            speed = self.robot.clamp_linear_speed(self.robot.kp_linear * distance, 0.0, 0.35)
            vx = speed * math.cos(relative_angle)
            vy = speed * math.sin(relative_angle)
            
            # 3. ROTATION LOCK (Maintain heading 0.0 rad)
            angular_err = angle_difference(0.0, -self.heading)
            angular_z = clamp(self.robot.kp_angular * angular_err, -0.6, 0.6)

            self.robot.drive(linear_x=vx, linear_y=vy, angular_z=angular_z)
            
            if now - self.last_log_time > 1.0:
                self.get_logger().info(f"WP {current_wp_idx+1}/{len(path)} | Dist: {distance:.2f}m | x:{self.pos_x:.2f} y:{self.pos_y:.2f}")
                self.last_log_time = now
                
            time.sleep(self.robot.poll_freq)

        self.robot.drive(0.0, 0.0, 0.0)
        self.get_logger().info("Target Reached!")
        return True

def main():
    rclpy.init()
    robot = Robot()
    time.sleep(2.0)
    
    import library.utils as utils
    team_id, robot_id = utils.get_team_robot_id()
    password = utils.get_password()
    comm = Communication("172.18.0.2:8000", team_id, robot_id, password)
    
    nav = GlobalNavigator(robot, comm)
    nav.wait_for_location()
    
    try:
        print("\n[*] Waiting for Pioneers to find an objective and map the route...")
        while rclpy.ok() and nav.target_goal is None:
            rclpy.spin_once(nav, timeout_sec=0.1)
            time.sleep(0.1)
        
        print(f"[*] Starting Expedition to: {nav.target_goal}")
        nav.navigate_to_target()
        
    except KeyboardInterrupt:
        pass
    finally:
        robot.drive(0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
