#!/usr/bin/env python3
import time

import rclpy
import random
import cv2

from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from search_aruco_action.action import SearchArUco


class SearchArucoActionServer(Node):

    def __init__(self):
        super().__init__('search_aruco_action_server')

        # ActionServer
        self._action_server = ActionServer(
            self,
            SearchArUco,
            'SearchAruco',
            self.execute_callback)

        # Subscribe to laser scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.subscription
        
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Velocity publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_distance = 0.0
        self.bridge = CvBridge()
        self.detected_markers = {}

        self.get_logger().info('SearchAruco Action Server is running...')

    def laser_callback(self, msg):
        # print('Current distance: "%s"' % msg.ranges[0])
        self.current_distance = min(msg.ranges[0], msg.ranges[-10], msg.ranges[10])
        
    def image_callback(self, msg):
        # Konwertuj obraz ROS do OpenCV
        # print('Image received')
        
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # cv2.imshow('frame', gray)
        # cv2.waitKey(1)

        # Wykrywanie markerów ArUco
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        # Jeśli wykryto markery, przetwórz ich pozycję i ID
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                corner = corners[i][0]

                # Wyznaczamy środek markera
                center_x = int((corner[0][0] + corner[2][0]) / 2)
                center_y = int((corner[0][1] + corner[2][1]) / 2)
                
                print('Detected marker ID: {0}, center: ({1}, {2})'.format(marker_id, center_x, center_y))

                # Zapisujemy wykryty marker
                self.detected_markers[marker_id] = (center_x, center_y)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Read action order
        timeout = goal_handle.request.timeout

        feedback_msg = SearchArUco.Feedback()
        feedback_msg.partial_locations_x = []
        feedback_msg.partial_locations_y = []

        start_time = time.time()

        msg = Twist()

        while time.time() - start_time < timeout:
            if self.current_distance < 1.0:  # If obstacle is closer than 100 cm
                # Rotate in a random direction
                msg.linear.x = 0.0
                msg.angular.z = random.choice([-1.0, 1.0])
                self.publisher_.publish(msg)
                time.sleep(random.random() * 3) 
            else:
                # Drive forward
                msg.linear.x = 0.5
                msg.angular.z = 0.0
                self.publisher_.publish(msg)

            feedback_msg.partial_locations_x = [self.current_distance]
            # self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_locations_x))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        # Stop at the end
        msg.linear.x = 0.0
        msg.angular.x = 0.0
        self.publisher_.publish(msg)

        goal_handle.succeed()

        result = SearchArUco.Result()
        result.locations_x = feedback_msg.partial_locations_x
        result.locations_y = feedback_msg.partial_locations_y
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