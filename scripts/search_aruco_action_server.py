#!/usr/bin/env python3
import time

import rclpy
import random
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

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
            self.listener_callback,
            10)
        self.subscription

        # Velocity publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_distance = 0.0

        self.get_logger().info('SearchAruco Action Server is running...')

    def listener_callback(self, msg):
        print('Current distance: "%s"' % msg.ranges[0])
        self.current_distance = msg.ranges[0]

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
            if self.current_distance < 1.0:  # If obstacle is closer than 40 cm
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
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_locations_x))
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