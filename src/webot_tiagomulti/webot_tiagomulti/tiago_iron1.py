import os
import sys
from tarfile import DEFAULT_FORMAT
import time
from tkinter import Y
from turtle import position
from typing import Tuple
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped, Pose, Point, PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor,ParameterType
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Range, CameraInfo, JointState, LaserScan
from threading import Thread
from webot_tiagomulti.webot_tiagomulti import robot_navigator
from robot_navigator import BehaviorNavigator, TaskResult

"""ask user for input of coordinates"""
"""a user interface"""
def coordinateX():
    while True:
        try:
            x1 = float(input("Please enter the x coordinates:"))
        except ValueError:
            print ("Invalid input")
            continue
        else: 
            break
        return x1
def coordinateY():
    while True:
        try:
            y1 = float(input("Please enter the y coordinates: "))
        except ValueError:
            print("Invalid Input")
            continue
        else:
            break
        return y1

class ControlTiago(Node):
    def __init__(self):
        super().__init__('tiagobot1_node')

        self.declare_parameter("robot_name", "tiago iron1")
        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value

        self.navigator = BehaviorNavigator()

        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 0.1
    def go_to_goal(self, x1, y1):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'()
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x1
        goal_pose.pose.position.y = y1
        goal_pose.pose.orientation.w = 1.0

        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isNavcomp():

            i = i+1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                print('estimated time of arrival:' + '{0:.0f}'.format(Duration.from_msg(feedback.time_remain).nanoseconds/1e9) + 'seconds.')

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds = 600):
                self.navigator.cancelNav()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('goal sucess')
        elif result == TaskResult.CANCELLED:
            print('goal cancelled')
        elif result == TaskResult.FAILED:
            print('Goal failed')
        else:
            print('invalid status')
    
def main(args =None):
    rclpy.init(args=args)
    tiagobot = ControlTiago()

    while rclpy.ok():

        try:
            x1 = coordinateX()
            y1 = coordinateY()

            tiagobot.go_to_goal(x1, y1)

        except KeyboardInterrupt:
            try:
                tiagobot.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            except SystemExit:
                os._exit(1)
if __name__=='__main__':
    main()


    


