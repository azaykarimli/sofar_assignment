
import sys
import time
from turtle import position
from typing import Tuple
import rclpy
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

MAX_RANGE = 0.1

class tiago_control(Node):
    
    def __init__(self):
        super.__init__('tiagobot')
        self._navigator = None
        self._current_path = None
        self._reached_goal = True
        self._is_stopped = True
        self._other_reached_goal = True

        # declaring parameter to get robot name, initial and final poses
        self.declare_parameter('robot_name', 'name')
        self.declare_parameter('initial_pose', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('goal_pose',descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))

        # obtaining the name of robot
        self._name = self. get_parameter('robot name').get_parameter_value().string_value

        # getting the initial and final pose
        initial_pose = array_to_pose(self.get_parameter('initial_pose').get_parameter_value().double_array_value)
        goal_pose = array_to_pose(self.get_parameter('goal_pose').get_parameter_value().double_array_value)
        self.poses = [initial_pose, goal_pose]

        # number of current goal reached
        self._goal_index = 0

        # publisher for path computed
        self._path_pub = self.create_publisher(Path, 'global_path', 10)
        # publisher for goal reached
        self._goal_pub = self.create_publisher(EmptyMsg, 'goal_reached', 10)
        # subscriber when one robot reached the goal
        self._goal_sub = self.create_subscription(EmptyMsg,'other_goal_reached', self._other_reached_goal, 10)

        #service for temporarily stopping the robot
        self.tempwait_srv = self.create_service(SetBool, ' toggle_wait', self.request_toggle_wait)

    def other_reached_goal(self, msg):
        self._other_reached_goal = True
        self.get_logger().info("["+self._name+"] a robot reached the goal")
        self.check_and_set_next_goal()

    def request_toggle_wait(self, request, response):
        if request.data == True :
            self.wait_on_current_path()
            self.get_logger().info("["+self._name+"] started to wait")
        else:
            self.follow_current_path_until_goal()
            self.get_logger().info("["+self._name+"] stopped waiting")
        
        response.success = True
        response.message = ""
        return response
    
    def create_navigator(self):
        # creating nav2 help
        self._navigator = BehaviorNavigator(self._name)
        # setting the initial pose
        
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._navigator.get_clock().now().to_msg()
        msg.pose = self._poses[0]
        self._navigator.setInitialPose(msg)

        self._navigator.waitUntilNav2Active()

    def check_and_set_next_goal(self):
        if not self._reached_goal or not self._other_reached_goal:
            return
        
        self._reached_goal = False
        self._other_reached_goal = False

        self._goal_index = (self._goal_index+1)%2

        self.create_new_path()
        self.follow_current_path_until_goal()

    def create_new_path(self):
        # new current pose
        cpose = PoseStamped()
        cpose.header.frame_id = 'map'
        cpose.header.stamp = self._navigator.get_clock().now().to_msg()
        cpose.pose = self._poses[(self._goal_index-1)%2]
        # new goal pose
        gpose = PoseStamped()
        gpose.header.frame_id = 'map'
        gpose.header.stamp = self._navigator.get_clock().now().to_msg()
        gpose.pose = self._poses[self._goal_index]

        self._current_path = self._navigator.getPath(cpose, gpose)

        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._navigator.get_clock().now().to_msg()
        msg.poses = self._current_path.poses
        self._path_pub.publish(msg)

        self.get_logger().info("["+self._name+"] published a new path")

    def wait_on_current_path(self):
        if self._is_stopped:
            return False
        self._is_stopped =True
        self._navigator.cancelTask()

        self.destroy_timer(self._timer)
        return True

    def follow_current_path_until_goal(self):
        if not self._is_stopped:
            return False
        self._is_stopped = False
        self._navigator.followPath(self._current_path)

        self._timer = self.create_timer(1, self.goal_check_callback)
        self.counter = 0
        return True

    def goal_check_callback(self):
        if not self._navigator.isTaskComplete():
            self._counter += 1
            if self._counter % 3 == 0:
                self._navigator.clearLocalCostmap()
            return
        self.get_logger().info("["+self._name+"] Goal reached")

        self.destroy_timer(self._timer)
        self._is_stopped = True

        self._goal_pub.publish(EmptyMsg())
        self._reached_goal = True

        self.check_and_set_next_goal()
    
    def destroy_node(self):
        self._path_pub.destroy()
        self._goal_pub.destroy()
        self._goal_sub.destroy()
        self.tempwait_srv.destroy()
        self._navigator.destroy_node()
        super().destroy_node()
    
def array_to_pose(array):

    pose =Pose()
    pose.position.x = array[0]
    pose.position.y = array[1]
    pose.position.z = array[2]
    pose.orientation.x = array[3]
    pose.orientation.y = array[4]
    pose.orientation.z = array[5]
    pose.orientation.w = array[6]
    return pose

def main(args=None):
    rclpy.init(args=args)
    controller = tiago_control()
    controller.create_navigator()
    controller.check_and_set_next_goal()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.destroy_node()
        rclpy.shutdown()

if __name__== "__main__":
    main()






        

        
        
        
    

