from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
       Node(
           package='webot_tiagomulti',
           namespace='Tiago World',
           executable='tiagobot',
           name='tiago_iron',
           parameters=[
            {"robot_name": "tiago iron"},
           ]
       ),
       Node(
           package='webot_tiagomulti',
           namespace='Tiago World',
           executable='tiagobot1',
           name='tiago_iron1',
           parameters=[
            {"robot_name": "tiago iron1"},
           ]
       ),
   ])
