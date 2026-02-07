#!/usr/bin/env python3
"""
Task 2: ArUco-Based Navigation
Autonomously discovers pick/place locations using ArUco markers,
then navigates to them.

ArUco ID 26  -> Pick surface  (marker side = 25 cm)
ArUco ID 238 -> Place surface (marker side = 25 cm)

Prerequisites:
  Terminal 1: ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25
  Terminal 2: ros2 launch group25_tasks task2_navigation.launch.py map_path:=<path>
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Duration

import tf2_ros


# ArUco marker IDs for pick/place surfaces
PICK_MARKER_ID = 26
PLACE_MARKER_ID = 238
SURFACE_MARKER_SIZE = 0.25  # 25 cm


class ArucoNavigator(Node):

    def __init__(self):
        super().__init__('task2_aruco_navigator')
        self._cb_group = ReentrantCallbackGroup()
        self._bridge = CvBridge()

        # State
        self._camera_info = None
        self._latest_image = None
        self._found_markers = {}  # {marker_id: (x, y) in map frame}
        self._pick_reached = False
        self._place_reached = False

        # TF buffer for frame transforms
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscribers
        self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self._image_callback,
            10)
        self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',
            self._camera_info_callback,
            10)

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, '/nav_vel', 10)
        self._head_pub = self.create_publisher(
            JointTrajectory, '/head_controller/joint_trajectory', 10)

        # Nav2 action client
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group)

        # ArUco detector setup
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_ARUCO_ORIGINAL)
        self._aruco_params = cv2.aruco.DetectorParameters()

        self.get_logger().info('Task 2 ArUco Navigator initialized')

    def _camera_info_callback(self, msg):
        self._camera_info = msg

    def _image_callback(self, msg):
        self._latest_image = msg

    def _move_head(self, pan, tilt):
        """Move TIAGo's head to the given pan/tilt angles (radians)."""
        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points = [point]
        self._head_pub.publish(traj)
        time.sleep(1.5)

    def _detect_aruco_markers(self):
        """Detect ArUco markers in the current camera image.
        Returns list of (marker_id, tvec, rvec) tuples."""
        if self._latest_image is None or self._camera_info is None:
            return []

        cv_image = self._bridge.imgmsg_to_cv2(
            self._latest_image, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params)

        if ids is None:
            return []

        # Camera intrinsics
        k = np.array(self._camera_info.k).reshape(3, 3)
        d = np.array(self._camera_info.d)

        results = []
        for i, marker_id in enumerate(ids.flatten()):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i:i+1], SURFACE_MARKER_SIZE, k, d)
            results.append((int(marker_id), tvecs[0][0], rvecs[0][0]))
            self.get_logger().info(
                f'Detected ArUco ID {marker_id} at distance '
                f'{np.linalg.norm(tvecs[0][0]):.2f}m')

        return results

    def _get_marker_position_in_map(self, tvec):
        """Transform a marker position from camera frame to map frame.
        Returns (x, y) in map frame or None if transform fails."""
        try:
            transform = self._tf_buffer.lookup_transform(
                'map',
                'head_front_camera_rgb_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))

            # Marker position in camera optical frame
            # tvec = [right, down, forward] in camera optical frame
            cam_x, cam_y, cam_z = tvec

            # Get transform components
            t = transform.transform.translation
            q = transform.transform.rotation

            # Simple quaternion rotation (using rotation matrix)
            # Convert quaternion to rotation matrix
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            rot = np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]
            ])

            point_cam = np.array([cam_x, cam_y, cam_z])
            point_map = rot @ point_cam + np.array([t.x, t.y, t.z])

            return (float(point_map[0]), float(point_map[1]))

        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def _navigate_to(self, x, y, yaw=0.0, timeout=90.0):
        """Navigate to a position in the map frame."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Navigating to ({x:.2f}, {y:.2f})...')

        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=timeout)

        if result_future.result() is not None:
            self.get_logger().info('Navigation goal reached!')
            return True
        else:
            self.get_logger().warn('Navigation timed out')
            goal_handle.cancel_goal_async()
            return False

    def _rotate_and_search(self):
        """Rotate in place while scanning for ArUco markers."""
        self.get_logger().info('Rotating to search for ArUco markers...')

        # Look slightly down to see the surface markers on the tables
        self._move_head(0.0, -0.3)
        time.sleep(0.5)

        # Rotate 360 degrees in steps, checking for markers at each step
        twist = Twist()
        steps = 12  # 30 degrees per step
        for step in range(steps):
            # Check for markers
            rclpy.spin_once(self, timeout_sec=0.5)
            detections = self._detect_aruco_markers()

            for marker_id, tvec, rvec in detections:
                if marker_id in (PICK_MARKER_ID, PLACE_MARKER_ID):
                    pos = self._get_marker_position_in_map(tvec)
                    if pos is not None:
                        self._found_markers[marker_id] = pos
                        name = 'PICK' if marker_id == PICK_MARKER_ID else 'PLACE'
                        self.get_logger().info(
                            f'Found {name} marker (ID {marker_id}) '
                            f'at map position ({pos[0]:.2f}, {pos[1]:.2f})')

            # Rotate a step
            twist.angular.z = 0.5
            for _ in range(10):
                self._cmd_vel_pub.publish(twist)
                time.sleep(0.1)

            # Brief stop for stable detection
            twist.angular.z = 0.0
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.5)

        # Stop and reset head
        twist.angular.z = 0.0
        self._cmd_vel_pub.publish(twist)
        self._move_head(0.0, 0.0)

    def _navigate_to_marker_vicinity(self, marker_pos, offset_distance=1.0):
        """Navigate to a position offset from the marker
        (so the robot stops facing the marker, not on top of it)."""
        mx, my = marker_pos

        # Get robot's current position to determine approach direction
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
        except Exception:
            rx, ry = 0.0, 0.0

        # Compute approach point (offset_distance away from marker)
        dx = rx - mx
        dy = ry - my
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > 0.01:
            dx /= dist
            dy /= dist
        else:
            dx, dy = 1.0, 0.0

        target_x = mx + dx * offset_distance
        target_y = my + dy * offset_distance
        target_yaw = math.atan2(my - target_y, mx - target_x)

        return self._navigate_to(target_x, target_y, target_yaw)

    def run(self):
        """Main task 2 routine."""
        self.get_logger().info('Waiting for Nav2...')
        self._nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready!')

        # Wait for AMCL localization to converge
        self.get_logger().info(
            'Waiting 10s for AMCL localization to converge...')
        time.sleep(10.0)

        # Search positions to scan from (spread across the environment)
        search_positions = [
            (0.0, -3.0, 0.0),
            (1.5, -5.0, -1.57),
            (3.0, -6.5, 0.0),
            (1.0, -8.0, 1.57),
        ]

        # Search for both markers
        for sx, sy, syaw in search_positions:
            if (PICK_MARKER_ID in self._found_markers and
                    PLACE_MARKER_ID in self._found_markers):
                break

            self.get_logger().info(
                f'Moving to search position ({sx}, {sy})...')
            self._navigate_to(sx, sy, syaw)
            time.sleep(1.0)
            self._rotate_and_search()

        # Report findings
        if PICK_MARKER_ID in self._found_markers:
            pos = self._found_markers[PICK_MARKER_ID]
            self.get_logger().info(
                f'PICK location (ArUco {PICK_MARKER_ID}): '
                f'({pos[0]:.2f}, {pos[1]:.2f})')
        else:
            self.get_logger().error('PICK marker not found!')

        if PLACE_MARKER_ID in self._found_markers:
            pos = self._found_markers[PLACE_MARKER_ID]
            self.get_logger().info(
                f'PLACE location (ArUco {PLACE_MARKER_ID}): '
                f'({pos[0]:.2f}, {pos[1]:.2f})')
        else:
            self.get_logger().error('PLACE marker not found!')

        # Navigate to pick location
        if PICK_MARKER_ID in self._found_markers:
            self.get_logger().info('=== Navigating to PICK location ===')
            self._navigate_to_marker_vicinity(
                self._found_markers[PICK_MARKER_ID], offset_distance=0.8)
            self._pick_reached = True

        # Navigate to place location
        if PLACE_MARKER_ID in self._found_markers:
            self.get_logger().info('=== Navigating to PLACE location ===')
            self._navigate_to_marker_vicinity(
                self._found_markers[PLACE_MARKER_ID], offset_distance=0.8)
            self._place_reached = True

        # Summary
        self.get_logger().info('========================================')
        self.get_logger().info('TASK 2 COMPLETE')
        self.get_logger().info(
            f'  Pick location reached: {self._pick_reached}')
        self.get_logger().info(
            f'  Place location reached: {self._place_reached}')
        self.get_logger().info('========================================')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
