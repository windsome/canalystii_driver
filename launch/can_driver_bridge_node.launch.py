
# Copyright 2021 The Autoware Foundation
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.event_handlers.on_shutdown import OnShutdown
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from lifecycle_msgs.msg import Transition
from launch.actions import LogInfo

def generate_launch_description():
    share_dir = get_package_share_directory('canalystii_driver')
    node_name = 'can_bridge_node'

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'canalystii.param.yaml'),
        description='File path to the ROS2 parameters file to use'
    )

    DeclareLaunchArgument('csv_path',
                          default_value=os.path.join(share_dir, 'config', 'can1.csv'),
                          description='can report record'
    )

    bridge_node = LifecycleNode(
        package='canalystii_driver',
        executable='can_bridge_node_exe',
        name=node_name,
        namespace=TextSubstitution(text=''),
        parameters=[
            {'csv_path': LaunchConfiguration('csv_path', default=os.path.join(share_dir, 'config', 'can1.csv'))},
            LaunchConfiguration('params_file'),
        ],
        output='screen',
        remappings=[
            ('to_can_bus', '/canalystii/to_can_bus'),
            ('from_can_bus', '/canalystii/from_can_bus'),
        ],
        # arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=bridge_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(bridge_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        )
    )

    activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=bridge_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(bridge_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        )
    )

    shutdown_event_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                LogInfo(msg="[LifecycleLaunch] can driver node is to shutdown."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_node_name(node_name),
                        # lifecycle_node_matcher=matches_action(bridge_node),
                        transition_id=Transition.TRANSITION_ACTIVE_SHUTDOWN,
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        params_declare,
        bridge_node,
        configure_event_handler,
        activate_event_handler,
        shutdown_event_handler,
    ])
