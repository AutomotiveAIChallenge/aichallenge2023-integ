# Copyright 2021 Tier IV, Inc. All rights reserved.
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

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def launch_setup(context, *args, **kwargs):
    # vehicle information param path
    vehicle_info_param_path = LaunchConfiguration("vehicle_param_file").perform(context)
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # planning common param path
    with open(LaunchConfiguration("common_param_path").perform(context), "r") as f:
        common_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # nearest search parameter
    with open(LaunchConfiguration("nearest_search_param_path").perform(context), "r") as f:
        nearest_search_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # obstacle avoidance planner
    with open(
        LaunchConfiguration("obstacle_avoidance_planner_param_path").perform(context), "r"
    ) as f:
        obstacle_avoidance_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    obstacle_avoidance_planner_component = ComposableNode(
        package="obstacle_avoidance_planner",
        plugin="obstacle_avoidance_planner::ObstacleAvoidancePlanner",
        name="obstacle_avoidance_planner",
        namespace="",
        remappings=[
            ("~/input/path", LaunchConfiguration("input_path_topic")),
            ("~/input/odometry", "/localization/kinematic_state"),
            ("~/output/path", "obstacle_avoidance_planner/trajectory"),
        ],
        parameters=[
            nearest_search_param,
            obstacle_avoidance_planner_param,
            vehicle_info_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # obstacle velocity limiter
    with open(
        LaunchConfiguration("obstacle_velocity_limiter_param_path").perform(context), "r"
    ) as f:
        obstacle_velocity_limiter_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    obstacle_velocity_limiter_component = ComposableNode(
        package="obstacle_velocity_limiter",
        plugin="obstacle_velocity_limiter::ObstacleVelocityLimiterNode",
        name="obstacle_velocity_limiter",
        namespace="",
        remappings=[
            ("~/input/trajectory", "obstacle_avoidance_planner/trajectory"),
            ("~/input/odometry", "/localization/kinematic_state"),
            ("~/input/dynamic_obstacles", "/perception/object_recognition/objects"),
            ("~/input/occupancy_grid", "/perception/occupancy_grid_map/map"),
            ("~/input/obstacle_pointcloud", "/perception/obstacle_segmentation/pointcloud"),
            ("~/input/map", "/map/vector_map"),
            ("~/output/debug_markers", "debug_markers"),
            ("~/output/trajectory", "obstacle_velocity_limiter/trajectory"),
        ],
        parameters=[
            obstacle_velocity_limiter_param,
            vehicle_info_param,
            {"obstacles.dynamic_source": "static_only"},
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # surround obstacle checker
    with open(
        LaunchConfiguration("surround_obstacle_checker_param_path").perform(context), "r"
    ) as f:
        surround_obstacle_checker_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    surround_obstacle_checker_component = ComposableNode(
        package="surround_obstacle_checker",
        plugin="surround_obstacle_checker::SurroundObstacleCheckerNode",
        name="surround_obstacle_checker",
        namespace="",
        remappings=[
            ("~/output/no_start_reason", "/planning/scenario_planning/status/no_start_reason"),
            ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
            ("~/output/max_velocity", "/planning/scenario_planning/max_velocity_candidates"),
            (
                "~/output/velocity_limit_clear_command",
                "/planning/scenario_planning/clear_velocity_limit",
            ),
            (
                "~/input/pointcloud",
                "/perception/obstacle_segmentation/pointcloud",
            ),
            ("~/input/objects", "/perception/object_recognition/objects"),
            ("~/input/odometry", "/localization/kinematic_state"),
        ],
        parameters=[
            surround_obstacle_checker_param,
            vehicle_info_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # obstacle stop planner
    with open(LaunchConfiguration("obstacle_stop_planner_param_path").perform(context), "r") as f:
        obstacle_stop_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(
        LaunchConfiguration("obstacle_stop_planner_acc_param_path").perform(context), "r"
    ) as f:
        obstacle_stop_planner_acc_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    obstacle_stop_planner_component = ComposableNode(
        package="obstacle_stop_planner",
        plugin="motion_planning::ObstacleStopPlannerNode",
        name="obstacle_stop_planner",
        namespace="",
        remappings=[
            ("~/output/stop_reason", "/planning/scenario_planning/status/stop_reason"),
            ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
            ("~/output/max_velocity", "/planning/scenario_planning/max_velocity_candidates"),
            (
                "~/output/velocity_limit_clear_command",
                "/planning/scenario_planning/clear_velocity_limit",
            ),
            ("~/output/trajectory", "/planning/scenario_planning/lane_driving/trajectory"),
            ("~/input/acceleration", "/localization/acceleration"),
            (
                "~/input/pointcloud",
                "/perception/obstacle_segmentation/pointcloud",
            ),
            ("~/input/objects", "/perception/object_recognition/objects"),
            ("~/input/odometry", "/localization/kinematic_state"),
            ("~/input/trajectory", "obstacle_velocity_limiter/trajectory"),
        ],
        parameters=[
            nearest_search_param,
            common_param,
            obstacle_stop_planner_param,
            obstacle_stop_planner_acc_param,
            vehicle_info_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # obstacle cruise planner
    with open(LaunchConfiguration("obstacle_cruise_planner_param_path").perform(context), "r") as f:
        obstacle_cruise_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    obstacle_cruise_planner_component = ComposableNode(
        package="obstacle_cruise_planner",
        plugin="motion_planning::ObstacleCruisePlannerNode",
        name="obstacle_cruise_planner",
        namespace="",
        remappings=[
            ("~/input/trajectory", "obstacle_velocity_limiter/trajectory"),
            ("~/input/odometry", "/localization/kinematic_state"),
            ("~/input/acceleration", "/localization/acceleration"),
            ("~/input/objects", "/perception/object_recognition/objects"),
            ("~/output/trajectory", "/planning/scenario_planning/lane_driving/trajectory"),
            ("~/output/velocity_limit", "/planning/scenario_planning/max_velocity_candidates"),
            ("~/output/clear_velocity_limit", "/planning/scenario_planning/clear_velocity_limit"),
            ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
        ],
        parameters=[
            nearest_search_param,
            common_param,
            obstacle_cruise_planner_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    obstacle_cruise_planner_relay_component = ComposableNode(
        package="topic_tools",
        plugin="topic_tools::RelayNode",
        name="obstacle_cruise_planner_relay",
        namespace="",
        parameters=[
            {"input_topic": "obstacle_velocity_limiter/trajectory"},
            {"output_topic": "/planning/scenario_planning/lane_driving/trajectory"},
            {"type": "autoware_auto_planning_msgs/msg/Trajectory"},
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name="motion_planning_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            obstacle_avoidance_planner_component,
            obstacle_velocity_limiter_component,
        ],
    )

    obstacle_stop_planner_loader = LoadComposableNodes(
        composable_node_descriptions=[obstacle_stop_planner_component],
        target_container=container,
        condition=LaunchConfigurationEquals("cruise_planner_type", "obstacle_stop_planner"),
    )

    obstacle_cruise_planner_loader = LoadComposableNodes(
        composable_node_descriptions=[obstacle_cruise_planner_component],
        target_container=container,
        condition=LaunchConfigurationEquals("cruise_planner_type", "obstacle_cruise_planner"),
    )

    obstacle_cruise_planner_relay_loader = LoadComposableNodes(
        composable_node_descriptions=[obstacle_cruise_planner_relay_component],
        target_container=container,
        condition=LaunchConfigurationEquals("cruise_planner_type", "none"),
    )

    surround_obstacle_checker_loader = LoadComposableNodes(
        composable_node_descriptions=[surround_obstacle_checker_component],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("use_surround_obstacle_check")),
    )

    group = GroupAction(
        [
            container,
            obstacle_stop_planner_loader,
            obstacle_cruise_planner_loader,
            obstacle_cruise_planner_relay_loader,
            surround_obstacle_checker_loader,
        ]
    )
    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # vehicle parameter
    add_launch_arg("vehicle_param_file")

    # interface parameter
    add_launch_arg(
        "input_path_topic",
        "/planning/scenario_planning/lane_driving/behavior_planning/path",
        "input path topic of obstacle_avoidance_planner",
    )

    # package parameter
    add_launch_arg("use_surround_obstacle_check")  # launch surround_obstacle_checker or not
    add_launch_arg(
        "cruise_planner_type"
    )  # select from "obstacle_stop_planner", "obstacle_cruise_planner", "none"

    add_launch_arg("use_intra_process", "false", "use ROS2 component container communication")
    add_launch_arg("use_multithread", "false", "use multithread")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )
    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [
            set_container_executable,
            set_container_mt_executable,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
