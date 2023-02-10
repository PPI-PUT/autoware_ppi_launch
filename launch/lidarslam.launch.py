# Copyright 2023 Perception for Physical Interaction Laboratory at Poznan University of Technology
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

import lifecycle_msgs.msg

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState, matches_node_name
from launch_ros.event_handlers import OnStateTransition


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare(LaunchConfiguration('param_file_pkg'))
    slam_config = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('slam_param_file')])
    ouster_config = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('ouster_param_file')])
    urdf_config = PathJoinSubstitution([pkg_prefix, 'urdf/sensors.xacro'])
    rviz_config = PathJoinSubstitution([pkg_prefix, 'rviz/mapping.rviz'])
    
    ouster_node = LifecycleNode(package='ros2_ouster',
                                executable='ouster_driver',
                                name='ouster_driver',
                                output='screen',
                                emulate_tty=True,
                                parameters=[ouster_config],
                                arguments=['--ros-args', '--log-level', 'INFO'],
                                namespace='/',
                                )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ouster_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ouster_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(ouster_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # TODO make lifecycle transition to shutdown before SIGINT
    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                  lifecycle_node_matcher=matches_node_name(node_name='ouster_driver'),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is exiting."),
            ],
        )
    )

    mapping_node = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[slam_config],
        remappings=[('/input_cloud','/points')],
        output='screen'
        )

    graphbasedslam_node = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[slam_config],
        output='screen'
        )
    
    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='sensors_state_publisher',
        namespace='sensors',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro', ' ', urdf_config])
        }]
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_config.perform(context))],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )
    
    return [
        ouster_node,
        configure_event,
        activate_event,
        shutdown_event,
        mapping_node,
        graphbasedslam_node,
        state_publisher_node,
        rviz2
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'param_file_pkg',
            default_value='autoware_ppi_launch',
            description="Package name which contains param file."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'slam_param_file',
            default_value='param/lidarslam.param.yaml',
            description="Param file (relative path)."
        )
    )
    

    declared_arguments.append(
        DeclareLaunchArgument(
            'ouster_param_file',
            default_value='param/ouster.param.yaml',
            description="Param file (relative path)."
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'with_rviz',
            default_value='True',
            description='Run rviz.'
        )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
