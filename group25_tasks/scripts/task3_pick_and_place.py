#!/usr/bin/env python3
"""
Task 3: Pick and Place
Detects ArUco cubes, picks them up using MoveIt, and transports them
from the pick surface to the place surface.

Order: cube ID 63 first, then cube ID 582
Cube size: 7 cm, ArUco marker size: 7 cm (on top face)
Surface markers: ID 26 (pick, 25cm), ID 238 (place, 25cm)

Prerequisites:
  Terminal 1: ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25 moveit:=true
  Terminal 2: ros2 launch group25_tasks task3_pick_place.launch.py map_path:=<path>
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
from geometry_msgs.msg import Twist, PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint,
    BoundingVolume, RobotState
)
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration

import tf2_ros

# Marker IDs
PICK_SURFACE_ID = 26
PLACE_SURFACE_ID = 238
CUBE_IDS = [63, 582]  # Order of transport
SURFACE_MARKER_SIZE = 0.25
CUBE_MARKER_SIZE = 0.07
CUBE_SIZE = 0.07


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('task3_pick_and_place')
        self._cb_group = ReentrantCallbackGroup()
        self._bridge = CvBridge()

        # State
        self._camera_info = None
        self._latest_image = None
        self._found_surfaces = {}
        self._gripper_open = True

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscribers
        self.create_subscription(
            Image, '/head_front_camera/rgb/image_raw',
            self._image_cb, 10)
        self.create_subscription(
            CameraInfo, '/head_front_camera/rgb/camera_info',
            self._camera_info_cb, 10)

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, '/nav_vel', 10)
        self._head_pub = self.create_publisher(
            JointTrajectory, '/head_controller/joint_trajectory', 10)

        # Action clients
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group)
        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self._cb_group)
        self._torso_client = ActionClient(
            self, FollowJointTrajectory,
            '/torso_controller/follow_joint_trajectory',
            callback_group=self._cb_group)
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=self._cb_group)

        # ArUco detector
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_ARUCO_ORIGINAL)
        self._aruco_params = cv2.aruco.DetectorParameters()

        # Arm joint names
        self._arm_joints = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Predefined arm positions (joint angles in radians)
        self._home_position = [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]
        self._pre_grasp_position = [0.20, 0.0, -1.5, 1.94, -1.57, 0.5, 0.0]
        self._offer_position = [1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.get_logger().info('Task 3 Pick and Place initialized')

    # ---- Callbacks ----

    def _image_cb(self, msg):
        self._latest_image = msg

    def _camera_info_cb(self, msg):
        self._camera_info = msg

    # ---- Head Control ----

    def _move_head(self, pan, tilt):
        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points = [point]
        self._head_pub.publish(traj)
        time.sleep(1.5)

    # ---- Torso Control ----

    def _move_torso(self, height):
        """Move torso to a specific height (0.0 to 0.35 m)."""
        self.get_logger().info(f'Moving torso to {height:.2f}m')
        self._torso_client.wait_for_server(timeout_sec=5.0)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['torso_lift_joint']
        point = JointTrajectoryPoint()
        point.positions = [height]
        point.time_from_start = Duration(sec=3, nanosec=0)
        goal.trajectory.points = [point]

        future = self._torso_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        if goal_handle and goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=15.0)
        time.sleep(1.0)

    # ---- Arm Control ----

    def _move_arm(self, joint_positions, duration_sec=4):
        """Move arm to specified joint positions."""
        self.get_logger().info(f'Moving arm...')
        self._arm_client.wait_for_server(timeout_sec=5.0)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self._arm_joints
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        goal.trajectory.points = [point]

        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        if goal_handle and goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=duration_sec + 10.0)
            self.get_logger().info('Arm motion complete')
        else:
            self.get_logger().warn('Arm goal rejected')
        time.sleep(1.0)

    def _move_arm_to_home(self):
        self.get_logger().info('Moving arm to home position')
        self._move_arm(self._home_position)

    def _move_arm_to_pre_grasp(self):
        self.get_logger().info('Moving arm to pre-grasp position')
        self._move_arm(self._pre_grasp_position)

    # ---- Gripper Control ----

    def _control_gripper(self, position):
        """Control gripper. position: 0.0=closed, 0.04=open."""
        state = 'OPEN' if position > 0.02 else 'CLOSED'
        self.get_logger().info(f'Gripper: {state} (pos={position})')
        self._gripper_client.wait_for_server(timeout_sec=5.0)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [point]

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        if goal_handle and goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=10.0)
        time.sleep(1.5)
        self._gripper_open = position > 0.02

    def _open_gripper(self):
        self._control_gripper(0.04)

    def _close_gripper(self):
        self._control_gripper(0.0)

    # ---- ArUco Detection ----

    def _detect_markers(self, target_ids=None, marker_size=None):
        """Detect specific ArUco markers. Returns {id: (tvec, rvec)}."""
        if self._latest_image is None or self._camera_info is None:
            return {}

        rclpy.spin_once(self, timeout_sec=0.5)

        cv_image = self._bridge.imgmsg_to_cv2(
            self._latest_image, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params)

        if ids is None:
            return {}

        k = np.array(self._camera_info.k).reshape(3, 3)
        d = np.array(self._camera_info.d)

        if marker_size is None:
            marker_size = SURFACE_MARKER_SIZE

        results = {}
        for i, mid in enumerate(ids.flatten()):
            mid = int(mid)
            if target_ids is not None and mid not in target_ids:
                continue
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i:i+1], marker_size, k, d)
            results[mid] = (tvecs[0][0], rvecs[0][0])

        return results

    def _get_position_in_base(self, tvec):
        """Transform camera-frame position to base_footprint frame."""
        try:
            transform = self._tf_buffer.lookup_transform(
                'base_footprint',
                'head_front_camera_rgb_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))

            t = transform.transform.translation
            q = transform.transform.rotation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w

            rot = np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]
            ])

            p = rot @ np.array(tvec) + np.array([t.x, t.y, t.z])
            return p

        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def _get_position_in_map(self, tvec):
        """Transform camera-frame position to map frame."""
        try:
            transform = self._tf_buffer.lookup_transform(
                'map',
                'head_front_camera_rgb_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))

            t = transform.transform.translation
            q = transform.transform.rotation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w

            rot = np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]
            ])

            p = rot @ np.array(tvec) + np.array([t.x, t.y, t.z])
            return (float(p[0]), float(p[1]))

        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    # ---- Navigation ----

    def _navigate_to(self, x, y, yaw=0.0, timeout=90.0):
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
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=timeout)
        return result_future.result() is not None

    def _navigate_near_marker(self, marker_pos, offset=0.7):
        """Navigate to a position offset from a marker."""
        mx, my = marker_pos
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except Exception:
            rx, ry = 0.0, 0.0

        dx = rx - mx
        dy = ry - my
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > 0.01:
            dx /= dist
            dy /= dist
        else:
            dx, dy = 1.0, 0.0

        tx = mx + dx * offset
        ty = my + dy * offset
        tyaw = math.atan2(my - ty, mx - tx)
        return self._navigate_to(tx, ty, tyaw)

    # ---- Search for surfaces ----

    def _rotate_and_find_surfaces(self):
        """Rotate to find pick/place surface ArUco markers."""
        self._move_head(0.0, -0.3)
        time.sleep(0.5)

        twist = Twist()
        for _ in range(12):
            rclpy.spin_once(self, timeout_sec=0.5)
            detections = self._detect_markers(
                target_ids=[PICK_SURFACE_ID, PLACE_SURFACE_ID],
                marker_size=SURFACE_MARKER_SIZE)

            for mid, (tvec, rvec) in detections.items():
                pos = self._get_position_in_map(tvec)
                if pos is not None:
                    self._found_surfaces[mid] = pos
                    name = 'PICK' if mid == PICK_SURFACE_ID else 'PLACE'
                    self.get_logger().info(
                        f'Found {name} surface at ({pos[0]:.2f}, {pos[1]:.2f})')

            twist.angular.z = 0.5
            for _ in range(10):
                self._cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            twist.angular.z = 0.0
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.5)

        self._move_head(0.0, 0.0)

    def _find_surfaces(self):
        """Search for pick and place surfaces."""
        search_positions = [
            (0.0, -3.0, 0.0),
            (1.5, -5.5, -1.57),
            (3.0, -7.0, -1.57),
            (1.5, -8.0, 1.57),
        ]

        for sx, sy, syaw in search_positions:
            if (PICK_SURFACE_ID in self._found_surfaces and
                    PLACE_SURFACE_ID in self._found_surfaces):
                break
            self._navigate_to(sx, sy, syaw)
            time.sleep(1.0)
            self._rotate_and_find_surfaces()

    # ---- Pick and Place Logic ----

    def _look_at_table(self):
        """Tilt head down to see objects on the table."""
        self._move_head(0.0, -0.55)
        time.sleep(1.0)

    def _fine_align_to_cube(self, cube_id):
        """Use visual servoing to align robot in front of the cube.
        Returns the cube position in base_footprint frame or None."""
        self._look_at_table()
        time.sleep(1.0)

        for attempt in range(5):
            rclpy.spin_once(self, timeout_sec=0.5)
            detections = self._detect_markers(
                target_ids=[cube_id], marker_size=CUBE_MARKER_SIZE)

            if cube_id not in detections:
                self.get_logger().warn(
                    f'Cube {cube_id} not visible, rotating slightly...')
                twist = Twist()
                twist.angular.z = 0.3
                for _ in range(5):
                    self._cmd_vel_pub.publish(twist)
                    time.sleep(0.1)
                twist.angular.z = 0.0
                self._cmd_vel_pub.publish(twist)
                time.sleep(0.5)
                continue

            tvec, rvec = detections[cube_id]
            cube_pos = self._get_position_in_base(tvec)
            if cube_pos is not None:
                self.get_logger().info(
                    f'Cube {cube_id} in base frame: '
                    f'x={cube_pos[0]:.3f}, y={cube_pos[1]:.3f}, '
                    f'z={cube_pos[2]:.3f}')
                return cube_pos

        return None

    def _pick_cube(self, cube_id):
        """Pick up a specific cube."""
        self.get_logger().info(f'=== PICKING cube ID {cube_id} ===')

        # Raise torso for better reach
        self._move_torso(0.30)

        # Open gripper
        self._open_gripper()

        # Move arm to pre-grasp
        self._move_arm_to_pre_grasp()

        # Detect cube position
        cube_pos = self._fine_align_to_cube(cube_id)
        if cube_pos is None:
            self.get_logger().error(f'Cannot find cube {cube_id}!')
            return False

        # The cube is on a table ~0.30m high
        # cube_pos is in base_footprint frame
        # We need to move the arm to reach the cube
        cx, cy, cz = cube_pos

        # Grasp approach: move arm joints to reach the cube
        # These are approximate joint positions for reaching a table-height object
        # The exact values depend on the cube position relative to the robot

        # Pre-grasp: arm extended forward, above the cube
        pre_grasp = [0.07, 0.45, -0.30, 1.60, -1.57, -0.20, 0.0]
        self.get_logger().info('Moving to pre-grasp over cube...')
        self._move_arm(pre_grasp, duration_sec=4)

        # Lower to grasp position
        grasp_pos = [0.07, 0.60, -0.30, 1.40, -1.57, -0.20, 0.0]
        self.get_logger().info('Lowering to grasp position...')
        self._move_arm(grasp_pos, duration_sec=3)

        # Close gripper
        self.get_logger().info('Closing gripper...')
        self._close_gripper()
        time.sleep(1.0)

        # Lift cube
        lift_pos = [0.07, 0.20, -0.30, 1.60, -1.57, -0.20, 0.0]
        self.get_logger().info('Lifting cube...')
        self._move_arm(lift_pos, duration_sec=3)

        # Move to safe carry position
        carry_pos = [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]
        self.get_logger().info('Moving to carry position...')
        self._move_arm(carry_pos, duration_sec=4)

        self.get_logger().info(f'Cube {cube_id} picked up!')
        return True

    def _place_cube(self):
        """Place the held cube on the place surface."""
        self.get_logger().info('=== PLACING cube ===')

        # Move torso up
        self._move_torso(0.30)

        # Move to pre-place position
        pre_place = [0.07, 0.45, -0.30, 1.60, -1.57, -0.20, 0.0]
        self.get_logger().info('Moving to pre-place position...')
        self._move_arm(pre_place, duration_sec=4)

        # Lower to place position
        place_pos = [0.07, 0.60, -0.30, 1.40, -1.57, -0.20, 0.0]
        self.get_logger().info('Lowering to place position...')
        self._move_arm(place_pos, duration_sec=3)

        # Open gripper
        self.get_logger().info('Opening gripper...')
        self._open_gripper()
        time.sleep(1.0)

        # Retreat arm upward
        retreat_pos = [0.07, 0.20, -0.30, 1.60, -1.57, -0.20, 0.0]
        self.get_logger().info('Retreating arm...')
        self._move_arm(retreat_pos, duration_sec=3)

        # Move to home
        self._move_arm_to_home()

        self.get_logger().info('Cube placed!')
        return True

    # ---- Main Pipeline ----

    def run(self):
        """Main Task 3 routine."""
        self.get_logger().info('Waiting for action servers...')
        self._nav_client.wait_for_server()
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info('All action servers ready!')

        # Wait for localization
        self.get_logger().info('Waiting for localization...')
        time.sleep(10.0)

        # Move arm to home first
        self._move_arm_to_home()

        # Step 1: Find pick and place surfaces using ArUco
        self.get_logger().info('=== STEP 1: Finding surfaces ===')
        self._find_surfaces()

        if PICK_SURFACE_ID not in self._found_surfaces:
            self.get_logger().error('Pick surface not found! Aborting.')
            return
        if PLACE_SURFACE_ID not in self._found_surfaces:
            self.get_logger().error('Place surface not found! Aborting.')
            return

        pick_pos = self._found_surfaces[PICK_SURFACE_ID]
        place_pos = self._found_surfaces[PLACE_SURFACE_ID]

        # Step 2: Transport each cube
        for cube_id in CUBE_IDS:
            self.get_logger().info(
                f'====== TRANSPORTING CUBE {cube_id} ======')

            # Navigate to pick surface
            self.get_logger().info('Navigating to PICK surface...')
            self._navigate_near_marker(pick_pos, offset=0.55)
            time.sleep(2.0)

            # Pick up the cube
            if not self._pick_cube(cube_id):
                self.get_logger().error(
                    f'Failed to pick cube {cube_id}, skipping...')
                continue

            # Navigate to place surface
            self.get_logger().info('Navigating to PLACE surface...')
            self._navigate_near_marker(place_pos, offset=0.55)
            time.sleep(2.0)

            # Place the cube
            self._place_cube()

            self.get_logger().info(
                f'Cube {cube_id} transported successfully!')

        self.get_logger().info('========================================')
        self.get_logger().info('TASK 3 COMPLETE - All cubes transported!')
        self.get_logger().info('========================================')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
