import sys
import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions.lifecycle_node import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher


PACKAGE_NAME = 'webot_tiagomulti'
def generate_launch_description():
    optional_nodes = []
    package_dir = get_package_share_directory(PACKAGE_NAME)
    tiago_iron_description = pathlib.Path(os.path.join(package_dir, 'resource', 'tiago_webots.urdf')).read_text()
    tiago_iron1_description = pathlib.Path(os.path.join(package_dir, 'resource', 'tiago_webots.urdf')).read_text()
    tiago_iron_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    tiago_iron1_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_nav = LaunchConfiguration('nav', default=True)
    use_slam = LaunchConfiguration('slam', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    nav2_map = os.path.join(package_dir, 'resource', 'map.yaml')
    # Webots
    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'default.wbt'))
    mode=mode

    # Driver nodes
    # When having multiple robot it is enough to specify the `additional_env` argument.
    # The `WEBOTS_ROBOT_NAME` has to match the robot name in the world file.
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'galactic'

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller', '-c', 'tiago_iron/controller_manager'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster', '-c', 'tiago_iron/controller_manager'] + controller_manager_timeout,
    )
    
    diffdrive_controller_spawner_ti1 = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller', '-c', 'tiago_iron1/controller_manager'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner_ti1 = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster', '-c', 'tiago_iron1/controller_manager'] + controller_manager_timeout,
    )
    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel')]
    if 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] in ['humble', 'rolling']:
        mappings.append(('/diffdrive_controller/odom', '/odom'))

    tiago_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'tiago iron'},
        namespace='tiago_iron',
        parameters=[
            {'robot_description': tiago_iron_description},
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
            tiago_iron_control_params
        ],
        remappings= mappings
        
    )

    tiago_controller = Node(
        package= 'webot_tiagomulti',
        namespace='tiago_iron',
        executable='tiagobot',
        name='tiago_iron',
        parameters=[
            {"robot_names": "tiago iron"},
           ]
       ),
    
    tiago1_controller = Node(
        package= 'webot_tiagomulti',
        namespace='tiago_iron1',
        executable='tiagobot',
        name='tiago_iron1',
        parameters=[
            {"robot_names": "tiago iron1"},
           ]
       ),
    
    tiago_iron1_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'tiago iron1'},
        namespace='tiago_iron1',
        parameters=[
            {'robot_description': tiago_iron1_description},
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
            tiago_iron1_control_params
        ],
        remappings= mappings
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        namespace='tiago_iron',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
           'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        namespace='tiago_iron1',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
           'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['1', '0', '0', '1', '0', '0', 'base_link', 'base_footprint'],
    )

    rviz_config = os.path.join(get_package_share_directory('webot_tiagomulti'), 'resource', 'default.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz),
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose'),])
    
    rviz1 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz),
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose'),])
    
    if 'nav2_bringup' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_nav)))

    slam_toolbox = Node(
        parameters=[{'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='tiago_iron',
        name='slam_toolbox',
        output='screen',
        condition=launch.conditions.IfCondition(use_slam)
    )

    slam_toolbox1 = Node(
        parameters=[{'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='tiago_iron1',
        name='slam_toolbox',
        output='screen',
        condition=launch.conditions.IfCondition(use_slam)
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='default.wbt',
            description='Choose one of the world files from `/webot_tiagomulti/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,
        rviz,
        rviz1,
        robot_state_publisher,
        slam_toolbox ,
        footprint_publisher,
        webots,
        tiago_driver,
        tiago_iron1_driver,
        diffdrive_controller_spawner_ti1,
        joint_state_broadcaster_spawner_ti1,
        tiago_controller,
        tiago1_controller,
        slam_toolbox1,
        
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )])