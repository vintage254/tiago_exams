#!/usr/bin/env python3
"""
Task 1: Autonomous SLAM Exploration
Drives TIAGo through waypoints to build a complete map of the group25 environment.

Usage:
  Terminal 1: ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25
  Terminal 2: ros2 launch group25_tasks task1_mapping.launch.py
  Terminal 3 (after exploration): ros2 run nav2_map_server map_saver_cli -f ~/my_map/map
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist


class ExplorationNode(Node):

    def __init__(self):
        super().__init__('task1_exploration')
        self._cb_group = ReentrantCallbackGroup()

        # Nav2 action client
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group)

        # Velocity publisher for initial rotation
        self._cmd_vel_pub = self.create_publisher(
            Twist, '/nav_vel', 10)

        # Waypoints covering the entire group25 environment
        # (x, y, yaw) - ordered to expand outward from spawn
        self.waypoints = [
            # Kitchen area (robot spawns at 0.0, -1.3)
            (0.0, -2.5, 0.0),
            (-1.5, -1.0, 1.57),
            (-2.0, -3.5, 3.14),
            (1.0, -3.5, -1.57),
            (1.0, -1.0, 0.0),
            # Transition to large room
            (1.5, -4.0, -1.57),
            (2.5, -5.0, -1.57),
            # Large room exploration
            (1.0, -6.0, 3.14),
            (0.5, -5.5, 0.0),
            (3.5, -5.5, 0.0),
            (5.0, -5.0, 0.0),
            (5.0, -7.0, -1.57),
            (3.0, -8.5, 3.14),
            (2.0, -8.5, 3.14),
            (1.0, -7.5, 1.57),
            # Return pass for coverage
            (0.5, -6.0, 0.0),
            (0.0, -3.0, 0.0),
        ]

        self.get_logger().info('Task 1 Exploration Node initialized')

    def initial_rotation(self):
        """Rotate in place to seed the SLAM map with initial laser data."""
        self.get_logger().info('Performing initial 360-degree rotation for SLAM seeding...')
        twist = Twist()
        twist.angular.z = 0.5  # rad/s

        # Rotate for ~13 seconds (~360 degrees at 0.5 rad/s)
        start = time.time()
        while time.time() - start < 13.0:
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        # Stop
        twist.angular.z = 0.0
        self._cmd_vel_pub.publish(twist)
        time.sleep(1.0)
        self.get_logger().info('Initial rotation complete')

    def wait_for_nav(self):
        """Wait for Nav2 action server to become available."""
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        while not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Still waiting for Nav2...')
        self.get_logger().info('Nav2 action server is ready!')

    def send_goal_and_wait(self, x, y, yaw, timeout=60.0):
        """Send a navigation goal and wait for result."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending goal: ({x:.1f}, {y:.1f}, yaw={yaw:.2f})')

        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

        if result_future.result() is not None:
            self.get_logger().info('Goal reached!')
            return True
        else:
            self.get_logger().warn('Goal timed out or failed')
            # Cancel the goal
            goal_handle.cancel_goal_async()
            time.sleep(1.0)
            return False

    def explore(self):
        """Main exploration routine."""
        self.wait_for_nav()

        # Wait for SLAM to initialize
        self.get_logger().info('Waiting 5s for SLAM to initialize...')
        time.sleep(5.0)

        # Initial rotation to seed the map
        self.initial_rotation()

        # Navigate through all waypoints
        total = len(self.waypoints)
        for i, (x, y, yaw) in enumerate(self.waypoints):
            self.get_logger().info(
                f'===== Waypoint {i+1}/{total} =====')
            success = self.send_goal_and_wait(x, y, yaw, timeout=90.0)
            if not success:
                self.get_logger().warn(
                    f'Failed to reach waypoint {i+1}, skipping...')
            # Pause for map update
            time.sleep(2.0)

        self.get_logger().info('========================================')
        self.get_logger().info('EXPLORATION COMPLETE!')
        self.get_logger().info('Save your map now with:')
        self.get_logger().info('  mkdir -p ~/my_map')
        self.get_logger().info(
            '  ros2 run nav2_map_server map_saver_cli -f ~/my_map/map')
        self.get_logger().info('========================================')


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        node.explore()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
