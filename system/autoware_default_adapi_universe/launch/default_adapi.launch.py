# Copyright 2022 TIER IV, Inc.
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

import pathlib

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def create_api_node(node_name, class_name):
    fullname = pathlib.Path("adapi/node") / node_name
    return ComposableNode(
        namespace=str(fullname.parent),
        name=str(fullname.name),
        package="autoware_default_adapi_universe",
        plugin="autoware::default_adapi::" + class_name,
        parameters=[ParameterFile(LaunchConfiguration("config"))],
    )


def get_default_config():
    path = FindPackageShare("autoware_default_adapi_universe")
    path = PathJoinSubstitution([path, "config/default_adapi.param.yaml"])
    return path


def generate_launch_description():
    components = [
        create_api_node("autoware_state", "AutowareStateNode"),
        create_api_node("diagnostics", "DiagnosticsNode"),
        create_api_node("fail_safe", "FailSafeNode"),
        create_api_node("heartbeat", "HeartbeatNode"),
        create_api_node("interface", "InterfaceNode"),
        create_api_node("localization", "LocalizationNode"),
        create_api_node("manual/local", "ManualControlNode"),
        create_api_node("manual/remote", "ManualControlNode"),
        create_api_node("motion", "MotionNode"),
        create_api_node("mrm_request", "MrmRequestNode"),
        create_api_node("operation_mode", "OperationModeNode"),
        create_api_node("perception", "PerceptionNode"),
        create_api_node("planning", "PlanningNode"),
        create_api_node("routing", "RoutingNode"),
        create_api_node("vehicle_status", "VehicleStatusNode"),
        create_api_node("vehicle_metrics", "VehicleMetricsNode"),
        create_api_node("vehicle_info", "VehicleInfoNode"),
        create_api_node("vehicle_door", "VehicleDoorNode"),
    ]
    container = ComposableNodeContainer(
        namespace="adapi",
        name="container",
        package="rclcpp_components",
        executable="component_container_mt",
        ros_arguments=["--log-level", "adapi.container:=WARN"],
        composable_node_descriptions=components,
    )
    argument = DeclareLaunchArgument("config", default_value=get_default_config())
    return launch.LaunchDescription([argument, container])
