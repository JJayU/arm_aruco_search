#!/usr/bin/env python3
import time

import rclpy
import random
import cv2
import numpy as np

from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan, Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from search_aruco_action.action import SearchArUco

from tf2_ros import Buffer, TransformListener


class SearchArucoActionServer(Node):

    def __init__(self):
        super().__init__('search_aruco_action_server')

        # ActionServer
        self._action_server = ActionServer(
            self,
            SearchArUco,
            'SearchAruco',
            self.execute_callback)

        # Laser subscription
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.subscription
        
        # Image subscription
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Camera info subscription
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)

        # Velocity publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_distance = 0.0
        self.bridge = CvBridge()
        self.detected_markers = {
            0: [0.0,0.0,0.0], 
            1: [0.0,0.0,0.0], 
            2: [0.0,0.0,0.0], 
            3: [0.0,0.0,0.0], 
            4: [0.0,0.0,0.0]          
        }

        self.marker_size = 0.07 # Rozmiar kostki - 9cm, sam marker ma 7x7

        self.camera_matrix = None
        self.dist_coeffs = None

        self.get_logger().info('SearchAruco Action Server is running...')

    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warn(f"Could not get transform from {source_frame} to {target_frame}: {e}")
            return None

    def laser_callback(self, msg):
        # Get distance to the closest obstacle in front of the robot
        self.current_distance = min((msg.ranges[-10:] + msg.ranges[:10]))

    def camera_info_callback(self, msg):
        # Get camera matrix and distortion coefficients
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def quaternion_to_rotation_matrix(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        rotation_matrix = np.array([
            [2 * (x**2 + y**2) - 1, 2 * (y*z - x*w)      , 2 * (y*w + x*z)      ],
            [2 * (y*z + x*w)      , 2 * (x**2 + z**2) - 1, 2 * (z*w - x*y)      ],
            [2 * (y*w - x*z)      , 2 * (z*w + x*y)      , 2 * (x**2 + w**2) - 1]
        ])
        return rotation_matrix
        
    def image_callback(self, msg):
        # print('Image received')
        
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ArUco marker detection
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        # Check if markers were detected
        if ids is not None and self.camera_matrix is not None and self.dist_coeffs is not None:
            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            # Get transform from map to camera_link
            transform = self.get_transform('map', 'camera_link')
            if transform is None:
                return

            # Get rotation matrix and translation vector
            rotation_matrix = self.quaternion_to_rotation_matrix(transform.transform.rotation)
            translation = np.array([transform.transform.translation.x,
                                     transform.transform.translation.y,
                                     transform.transform.translation.z])

            for i in range(len(ids)):
                marker_id = ids[i][0]
                position_camera = np.array(tvecs[i][0])
                position_camera = [-position_camera[2], position_camera[0], position_camera[1]]

                # Skip markers that are too far away
                if(max(position_camera) > 4.0):
                    continue

                # Transform position to relative to map
                position_world = np.dot(rotation_matrix, position_camera) + translation
                self.detected_markers[marker_id] = position_world

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Read action order
        timeout = goal_handle.request.timeout

        # Prepare feedback message
        feedback_msg = SearchArUco.Feedback()
        feedback_msg.partial_locations_x = []
        feedback_msg.partial_locations_y = []

        start_time = time.time()

        msg = Twist()

        while time.time() - start_time < timeout:
            # If obstacle is closer than 100 cm
            if self.current_distance < 1.0:  
                # Rotate in a random direction for random time
                msg.linear.x = 0.0
                msg.angular.z = random.choice([-1.0, 1.0])
                self.publisher_.publish(msg)
                time.sleep(random.random() * 3) 
            else:
                # Drive forward
                msg.linear.x = 0.5
                msg.angular.z = 0.0
                self.publisher_.publish(msg)

            # Send feedback message
            feedback_msg.partial_locations_x = [self.detected_markers[0][0], self.detected_markers[1][0], self.detected_markers[2][0], self.detected_markers[3][0], self.detected_markers[4][0]]
            feedback_msg.partial_locations_y = [self.detected_markers[0][1], self.detected_markers[1][1], self.detected_markers[2][1], self.detected_markers[3][1], self.detected_markers[4][1]]
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)

        # Stop at the end
        msg.linear.x = 0.0
        msg.angular.x = 0.0
        self.publisher_.publish(msg)
        
        # Prepare result
        goal_handle.succeed()
        result = SearchArUco.Result()
        result.locations_x = feedback_msg.partial_locations_x
        result.locations_y = feedback_msg.partial_locations_y
        result.succeeded = True
        if any(pos == 0.0 for pos in result.locations_x):
            result.succeeded = False

        return result


def main(args=None):
    rclpy.init(args=args)

    search_aruco_action_server = SearchArucoActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(search_aruco_action_server)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        search_aruco_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()