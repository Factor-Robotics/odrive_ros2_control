# Copyright 2021 Factor Robotics
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_joint0",
            default_value="true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_joint1",
            default_value="false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "joint0_controller",
            default_value="joint0_velocity_controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "joint1_controller",
            default_value="joint1_velocity_controller",
        )
    )

    enable_joint0 = LaunchConfiguration("enable_joint0")
    enable_joint1 = LaunchConfiguration("enable_joint1")
    joint0_controller = LaunchConfiguration("joint0_controller")
    joint1_controller = LaunchConfiguration("joint1_controller")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("odrive_description"),
                    "urdf",
                    "odrive.urdf.xacro",
                ]
            ),
            " ",
            "enable_joint0:=",
            enable_joint0,
            " ",
            "enable_joint1:=",
            enable_joint1,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_bringup"),
            "config",
            "odrive_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint0_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[joint0_controller, "-c", "/controller_manager"],
        condition=IfCondition(enable_joint0),
    )

    joint1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[joint1_controller, "-c", "/controller_manager"],
        condition=IfCondition(enable_joint1),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        joint0_controller_spawner,
        joint1_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
