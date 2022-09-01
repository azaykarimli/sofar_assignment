#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots and the controllers."""

import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher



PACKAGE_NAME = 'webots_ros2_universal_robot'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    tiago_iron_description = pathlib.Path(os.path.join(package_dir, 'resource', 'tiago_webots.urdf')).read_text()
    tiago_iron_description = pathlib.Path(os.path.join(package_dir, 'resource', 'tiago_webots.urdf')).read_text()
    tiago_iron_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    tiago_iron_control_params = os.path.join(package_dir, 'resource', 'ros2_control1.yml')

    # Webots
    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'default.wbt'))

    # Driver nodes
    # When having multiple robot it is enough to specify the `additional_env` argument.
    # The `WEBOTS_ROBOT_NAME` has to match the robot name in the world file.
    
    tiago_iron_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'tiago iron'},
        namespace='tiago_iron',
        parameters=[
            {'robot_description': tiagobot_description},
            {'use_sim_time': True},
            tiago_iron_control_params
        ]
    )
    tiago_iron(1)_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'tiago iron(1)'},
        namespace='tiago_iron(1)',
        parameters=[
            {'robot_description': tiagobot_description},
            {'use_sim_time': True},
            tiago_iron(1)_control_params
        ]
    )

    controller_manager_timeout = ['--controller-manager-timeout', '75']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'galactic'
    ur5e_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller', '-c', 'ur5e/controller_manager'] + controller_manager_timeout,
    )
    ur5e_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster', '-c', 'ur5e/controller_manager'] + controller_manager_timeout,
    )
    abb_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['abb_joint_trajectory_controller', '-c', 'abb/controller_manager'] + controller_manager_timeout,
    )
    abb_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['abb_joint_state_broadcaster', '-c', 'abb/controller_manager'] + controller_manager_timeout,
    )

    # Control nodes
    tiago_controller = Node(
        package=PACKAGE_NAME,
        executable='tiago_iron',
        namespace='tiago_iron',
        output='screen'
    )
    tiago1_controller = Node(
        package=PACKAGE_NAME,
        executable='tiago_iron1',
        namespace='tiago_iron(1)',
        output='screen'
    )

    return launch.LaunchDescription([
        webots,
        tiago1_controller,
        tiago_controller,
        tiago_iron_driver,
        tiago_iron(1)_driver,
        ur5e_trajectory_controller_spawner,
        ur5e_joint_state_broadcaster_spawner,
        abb_trajectory_controller_spawner,
        abb_joint_state_broadcaster_spawner,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
