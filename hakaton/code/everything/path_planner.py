import json
import math
import heapq
import numpy as np
import os

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

class AStarPlanner:
    def __init__(self, width, height, obstacles, resolution=0.1, robot_radius=0.35):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.obstacles = obstacles
        
        self.grid_width = int(width / resolution)
        self.grid_height = int(height / resolution)
        
        # Precompute occupancy grid with inflation
        self.grid = np.zeros((self.grid_height, self.grid_width), dtype=bool)
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                x = j * resolution + resolution/2
                y = i * resolution + resolution/2
                
                for obs in self.obstacles:
                    if is_point_in_poly(x, y, obs):
                        self.grid[i, j] = True
                        break
                    
                    # Inflation: distance to vertices
                    if any(math.hypot(x - p['x'], y - p['y']) < robot_radius for p in obs):
                        self.grid[i, j] = True
                        break
                    
                    # Optional: Add distance to segments for better inflation
                    # (Skipping for now to keep it lightweight)

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_neighbors(self, node):
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                if not self.grid[ny, nx]:
                    cost = math.sqrt(dx**2 + dy**2)
                    neighbors.append(((nx, ny), cost))
        return neighbors

    def plan(self, start_world, goal_world):
        start = (int(start_world[0] / self.resolution), int(start_world[1] / self.resolution))
        goal = (int(goal_world[0] / self.resolution), int(goal_world[1] / self.resolution))
        
        start = (max(0, min(self.grid_width-1, start[0])), max(0, min(self.grid_height-1, start[1])))
        goal = (max(0, min(self.grid_width-1, goal[0])), max(0, min(self.grid_height-1, goal[1])))

        if self.grid[start[1], start[0]]:
            print("[!] Warning: Start position is inside an obstacle inflation zone.")
        if self.grid[goal[1], goal[0]]:
            print("[!] Warning: Goal position is inside an obstacle inflation zone.")

        queue = [(0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0}

        while queue:
            current = heapq.heappop(queue)[1]
            if current == goal: break

            for next_node, weight in self.get_neighbors(current):
                new_cost = cost_so_far[current] + weight
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    heapq.heappush(queue, (priority, next_node))
                    came_from[next_node] = current

        if goal not in came_from:
            return None

        path = []
        curr = goal
        while curr is not None:
            path.append((curr[0] * self.resolution + self.resolution/2, 
                         curr[1] * self.resolution + self.resolution/2))
            curr = came_from[curr]
        path.reverse()
        return path
