#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
import tf2_ros

# Import your communication class
from library.communication import Communication  # adjust path if needed

class SimpleMapper(Node):

    def __init__(self):
        super().__init__('simple_mapper')

        # -----------------------------
        # Map parameters
        # -----------------------------
        self.resolution = 0.05  # meters per cell
        self.map_width_m = 6.0
        self.map_height_m = 7.5
        self.width = int(self.map_width_m / self.resolution)
        self.height = int(self.map_height_m / self.resolution)
        self.grid = np.zeros((self.height, self.width), dtype=np.float32)

        # Log-odds parameters
        self.l_occ = 0.7
        self.l_free = -0.4
        self.l_min = -2.0
        self.l_max = 3.5

        self.visible = False

        # -----------------------------
        # Robot pose (will be updated dynamically)
        # -----------------------------
        self.pose = (0.0, 0.0, 0.0)  # (x, y, theta)

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

        # -----------------------------
        # Static transform broadcaster (map -> odom)
        # -----------------------------
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_static_tf()

        self.get_logger().info("Simple Mapper (Top-Left Origin) Started")

        # -----------------------------
        # Communication client
        # -----------------------------
        self.comm = Communication(host="172.18.0.2:8000", team_id=3, robot_id=0, password="stagnant-attractor-companion")
        self.comm.register_callback_location(self.location_update)

    # -----------------------------
    # Callback from communication class
    # -----------------------------
    def location_update(self, x, y, angle, visible, last_seen):
        self.visible = visible
        if visible:
            self.pose = (x, y, angle)
            self.get_logger().debug(f"Pose updated: {self.pose}")

    # -----------------------------
    # Static TF broadcaster
    # -----------------------------
    def broadcast_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = self.euler_to_quaternion(0.0, 0.0, 0.0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform([t])

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
        qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
        qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        return [qx, qy, qz, qw]

    # -----------------------------
    # Coordinate conversion
    # -----------------------------
    def world_to_grid(self, x, y):
        gx = int(x / self.resolution)
        gy = int(y / self.resolution)
        return gx, gy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    # -----------------------------
    # Bresenham ray tracing
    # -----------------------------
    def bresenham(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        cells.append((x1, y1))
        return cells

    # -----------------------------
    # Map update
    # -----------------------------
    def update_map(self, scan):
        x, y, theta = self.pose
        robot_gx, robot_gy = self.world_to_grid(x, y)
        if not self.in_bounds(robot_gx, robot_gy):
            return

        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max:
                continue
            if r > 8.0:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            ex = x + r * math.sin(theta + angle)
            ey = y + r * math.cos(theta + angle)
            gx1, gy1 = self.world_to_grid(ex, ey)
            cells = self.bresenham(robot_gx, robot_gy, gx1, gy1)

            # Free cells
            for gx, gy in cells[:-1]:
                if self.in_bounds(gx, gy):
                    self.grid[gy, gx] = np.clip(
                        self.grid[gy, gx] + self.l_free,
                        self.l_min, self.l_max
                    )
            # Occupied cell
            if r < scan.range_max:
                gx, gy = cells[-1]
                if self.in_bounds(gx, gy):
                    self.grid[gy, gx] = np.clip(
                        self.grid[gy, gx] + self.l_occ,
                        self.l_min, self.l_max
                    )

    # -----------------------------
    # Convert to OccupancyGrid
    # -----------------------------
    def to_occupancy(self):
        prob = 1 - 1 / (1 + np.exp(self.grid))
        occ = np.full(self.grid.shape, -1, dtype=np.int8)
        occ[prob > 0.65] = 100
        occ[prob < 0.35] = 0
        return occ

    # -----------------------------
    # Publish the map
    # -----------------------------
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        # Bottom-left origin for ROS
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        occ = self.to_occupancy()
        # Flip vertically for ROS
        occ = np.fliplr(occ)
        # occ = np.flipud(occ)
        msg.data = occ.flatten().tolist()
        self.map_pub.publish(msg)

    # -----------------------------
    # LaserScan callback
    # -----------------------------
    def scan_callback(self, msg):
        # Only update the map if robot is visible
        if self.visible:
            self.update_map(msg)
            self.publish_map()
        else:
            self.get_logger().info("Skipping map update: robot not visible")

# -----------------------------
# Main
# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()