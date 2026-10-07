#!/usr/bin/env python3
"""
* Team Id: 2839
* Author List: SC#2839
* Filename: path_follower.py
* Theme: Kepler Colony / Kepler Arena (Task 1C)
* Functions: PathFollower, main
* Global Variables: INFLATION_RADIUS, LOOKAHEAD, V_MAX, etc.
"""

import math
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan


# --- Tuning Parameters ---
INFLATION_RADIUS = 0.32       # Safety obstacle inflation radius in meters
LOOKAHEAD = 0.40              # Pure pursuit lookahead distance (m)
V_MAX = 0.35                  # Maximum linear velocity (m/s)
V_MIN = 0.08                  # Minimum linear velocity (m/s)
W_MAX = 0.80                  # Maximum angular velocity (rad/s)
K_HEADING = 1.6               # Angular proportional gain
WP_ACCEPT_RADIUS = 0.28       # Intermediate waypoint threshold (< 0.5m)
FINAL_ACCEPT_RADIUS = 0.15    # Final waypoint stop threshold (m)
LIDAR_SAFETY_DIST = 0.35      # Collision avoidance trigger distance (m)
LIDAR_EMERGENCY_STOP = 0.22   # Emergency stop threshold (m)


def yaw_from_quaternion(q):
    """Calculate yaw angle from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.path_sub = self.create_subscription(Path, '/ebot_path', self.path_callback, latched_qos)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.waypoints = []
        self.current_wp_idx = 0
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = (0.0, 0.0)
        self.map_shape = (0, 0)
        self.inflated_grid = None

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        self.min_front_scan = float('inf')
        self.global_planned_path = []
        self.current_path_idx = 0
        self.navigation_complete = False

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('PathFollower Node Initialized.')

    def path_callback(self, msg: Path):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(f'Received route with {len(self.waypoints)} waypoints.')
        self.plan_entire_mission()

    def map_callback(self, msg: OccupancyGrid):
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_shape = (msg.info.height, msg.info.width)
        grid = np.array(msg.data, dtype=np.int8).reshape(self.map_shape)

        self.map_data = grid
        self.inflate_map()
        self.get_logger().info(f'Received map ({self.map_shape[1]}x{self.map_shape[0]}). Inflated successfully.')
        self.plan_entire_mission()

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        valid_ranges = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
                if -math.radians(30) <= angle <= math.radians(30):
                    valid_ranges.append(r)
            angle += msg.angle_increment

        self.min_front_scan = min(valid_ranges) if valid_ranges else float('inf')

    def inflate_map(self):
        """Pure NumPy binary obstacle dilation without requiring SciPy."""
        if self.map_data is None:
            return

        occ = (self.map_data >= 50) | (self.map_data == -1)
        r_cells = int(math.ceil(INFLATION_RADIUS / self.map_resolution))

        y, x = np.ogrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
        kernel = (x * x + y * y) <= (r_cells * r_cells)

        h, w = self.map_shape
        inflated = occ.copy()
        obstacle_y, obstacle_x = np.nonzero(occ)

        for oy, ox in zip(obstacle_y, obstacle_x):
            y_min, y_max = max(0, oy - r_cells), min(h, oy + r_cells + 1)
            x_min, x_max = max(0, ox - r_cells), min(w, ox + r_cells + 1)

            ky_min = y_min - (oy - r_cells)
            ky_max = ky_min + (y_max - y_min)
            kx_min = x_min - (ox - r_cells)
            kx_max = kx_min + (x_max - x_min)

            inflated[y_min:y_max, x_min:x_max] |= kernel[ky_min:ky_max, kx_min:kx_max]

        self.inflated_grid = inflated

    def world_to_grid(self, x, y):
        gx = int((x - self.map_origin[0]) / self.map_resolution)
        gy = int((y - self.map_origin[1]) / self.map_resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        x = (gx + 0.5) * self.map_resolution + self.map_origin[0]
        y = (gy + 0.5) * self.map_resolution + self.map_origin[1]
        return (x, y)

    def is_valid_cell(self, gx, gy):
        h, w = self.map_shape
        if 0 <= gx < w and 0 <= gy < h:
            return not self.inflated_grid[gy, gx]
        return False

    def a_star(self, start, goal):
        sx, sy = self.world_to_grid(start[0], start[1])
        gx, gy = self.world_to_grid(goal[0], goal[1])

        if not self.is_valid_cell(sx, sy) or not self.is_valid_cell(gx, gy):
            return [start, goal]

        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0.0}

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == (gx, gy):
                path = []
                curr = current
                while curr in came_from:
                    path.append(self.grid_to_world(curr[0], curr[1]))
                    curr = came_from[curr]
                path.append(start)
                path.reverse()
                path.append(goal)
                return self.smooth_path(path)

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.is_valid_cell(neighbor[0], neighbor[1]):
                    continue

                step_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                tentative_g = g_score[current] + step_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, (gx, gy))
                    heapq.heappush(open_set, (f_score, neighbor))

        return [start, goal]

    def smooth_path(self, path):
        if len(path) <= 2:
            return path
        smoothed = [path[0]]
        current_idx = 0
        while current_idx < len(path) - 1:
            next_idx = len(path) - 1
            while next_idx > current_idx + 1:
                if self.check_line_of_sight(path[current_idx], path[next_idx]):
                    break
                next_idx -= 1
            smoothed.append(path[next_idx])
            current_idx = next_idx
        return smoothed

    def check_line_of_sight(self, p1, p2):
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = max(int(dist / (self.map_resolution * 0.5)), 2)
        for i in range(steps + 1):
            t = i / steps
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            gx, gy = self.world_to_grid(x, y)
            if not self.is_valid_cell(gx, gy):
                return False
        return True

    def plan_entire_mission(self):
        if not self.waypoints or self.inflated_grid is None:
            return

        self.global_planned_path = []
        for i in range(len(self.waypoints) - 1):
            leg = self.a_star(self.waypoints[i], self.waypoints[i + 1])
            if self.global_planned_path:
                self.global_planned_path.extend(leg[1:])
            else:
                self.global_planned_path.extend(leg)

        self.current_wp_idx = 0
        self.current_path_idx = 0
        self.get_logger().info(f'Full trajectory planned: {len(self.global_planned_path)} nodes.')

    def control_loop(self):
        if not self.odom_received or not self.global_planned_path or self.navigation_complete:
            return

        twist = Twist()

        target_wp = self.waypoints[self.current_wp_idx]
        dist_to_wp = math.hypot(target_wp[0] - self.robot_x, target_wp[1] - self.robot_y)

        is_last_wp = (self.current_wp_idx == len(self.waypoints) - 1)
        acceptance_radius = FINAL_ACCEPT_RADIUS if is_last_wp else WP_ACCEPT_RADIUS

        if dist_to_wp < acceptance_radius:
            self.get_logger().info(f'Reached waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}')
            if is_last_wp:
                self.navigation_complete = True
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info('Mission Complete. Holding position.')
                return
            self.current_wp_idx += 1

        target_point = self.global_planned_path[-1]
        for i in range(self.current_path_idx, len(self.global_planned_path)):
            pt = self.global_planned_path[i]
            d = math.hypot(pt[0] - self.robot_x, pt[1] - self.robot_y)
            if d >= LOOKAHEAD:
                target_point = pt
                self.current_path_idx = i
                break

        dx = target_point[0] - self.robot_x
        dy = target_point[1] - self.robot_y
        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - self.robot_yaw)

        if self.min_front_scan < LIDAR_EMERGENCY_STOP:
            twist.linear.x = 0.0
            twist.angular.z = W_MAX * 0.7
            self.cmd_vel_pub.publish(twist)
            return

        speed_factor = max(0.2, math.cos(heading_error))
        if self.min_front_scan < LIDAR_SAFETY_DIST:
            speed_factor *= (self.min_front_scan / LIDAR_SAFETY_DIST)

        if abs(heading_error) > math.radians(45):
            twist.linear.x = 0.0
            twist.angular.z = math.copysign(W_MAX * 0.6, heading_error)
        else:
            v = V_MIN + (V_MAX - V_MIN) * speed_factor
            w = K_HEADING * heading_error
            w = max(-W_MAX, min(W_MAX, w))
            twist.linear.x = float(v)
            twist.angular.z = float(w)

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.cmd_vel_pub.publish(Twist())
        except Exception: pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
