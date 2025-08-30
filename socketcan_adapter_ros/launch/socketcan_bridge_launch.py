# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
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
# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Define args
    can_interface_arg = DeclareLaunchArgument('can_interface', default_value='can0')
    can_error_mask_arg = DeclareLaunchArgument('can_error_mask', default_value='0x1FFFFFFF')
    can_filter_list_arg = DeclareLaunchArgument('can_filter_list', default_value="['']")
    join_filters_arg = DeclareLaunchArgument('join_filters', default_value='false')
    receive_timeout_arg = DeclareLaunchArgument('receive_timeout_s', default_value='1.0')

    socketcan_bridge_node = LifecycleNode(
        package='socketcan_adapter_ros',
        executable='socketcan_bridge',
        name='socketcan_bridge',
        namespace='',
        parameters=[
            {
                'can_interface': LaunchConfiguration('can_interface'),
                'can_error_mask': LaunchConfiguration('can_error_mask'),
                'can_filter_list': ParameterValue(LaunchConfiguration('can_filter_list'), value_type=list[str]),
                'join_filters': LaunchConfiguration('join_filters'),
                'receive_timeout_s': LaunchConfiguration('receive_timeout_s'),
            }
        ],
        output='screen',
    )

    socketcan_bridge_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socketcan_bridge_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socketcan_bridge_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socketcan_bridge_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socketcan_bridge_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socketcan_bridge_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    return LaunchDescription([
        can_interface_arg,
        can_error_mask_arg,
        can_filter_list_arg,
        join_filters_arg,
        receive_timeout_arg,
        DeclareLaunchArgument('auto_configure', default_value='true'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        socketcan_bridge_node,
        socketcan_bridge_configure_event_handler,
        socketcan_bridge_activate_event_handler,
    ])
