#include "self_driving_controller.hpp"

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_system_msgs/msg/autoware_state.hpp"
#include "autoware_auto_vehicle_msgs/msg/engage.hpp"

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cstdio>

namespace {
  using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
  using Engage = autoware_auto_vehicle_msgs::msg::Engage;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  Engage createEngageMessage()
  {
    auto msg = Engage();
    msg.engage = true;
    return msg;
  }

  PoseStamped createGoalPoseMessage()
  {
    auto msg = PoseStamped();

    msg.header.frame_id = "map";
    msg.pose.position.x = 3702.773681640625;
    msg.pose.position.y = 73742.1796875;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.8623958339931682;
    msg.pose.orientation.w = 0.5062345558248941;
    return msg;
  }
}

SelfDrivingController::SelfDrivingController()
 : Node("self_driving_controller")
 , step_counter_(0)
{
  // Publishers
  engage_publisher =
    this->create_publisher<Engage>("output/engage", 1);
  goal_pos_publisher =
    this->create_publisher<PoseStamped>("output/goal", 1);

  // Subscribers
  state_subscriber = this->create_subscription<AutowareState>(
    "input/state", 1, std::bind(&SelfDrivingController::stateCallback, this, std::placeholders::_1));
}

void SelfDrivingController::stateCallback(const AutowareState& msg) {
  switch (msg.state) {
    case AutowareState::WAITING_FOR_ROUTE:
      if (step_counter_ != 0)
        break;

      RCLCPP_INFO(this->get_logger(), "[AIChallengeSample]: Publishing goal pose.");
      goal_pos_publisher->publish(createGoalPoseMessage());

      ++step_counter_;
      break;
    case AutowareState::PLANNING:
      RCLCPP_INFO(this->get_logger(), "[AIChallengeSample]: Planning...");
      break;
    case AutowareState::WAITING_FOR_ENGAGE:
      if (step_counter_ != 1)
        break;

      RCLCPP_INFO(this->get_logger(), "[AIChallengeSample]: Publishing engage message.");
      engage_publisher->publish(createEngageMessage());

      ++step_counter_;
      break;
    default:
      break;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfDrivingController>());
  rclcpp::shutdown();
  return 0;
}
