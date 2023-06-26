#ifndef SELF_DRIVING_CONTROLLER__SELF_DRIVING_CONTROLLER_HPP_
#define SELF_DRIVING_CONTROLLER__SELF_DRIVING_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_system_msgs/msg/autoware_state.hpp"
#include "autoware_auto_vehicle_msgs/msg/engage.hpp"

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cstdio>

class SelfDrivingController : public rclcpp::Node
{
public:
  SelfDrivingController();

private:
  using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
  using Engage = autoware_auto_vehicle_msgs::msg::Engage;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  void stateCallback(const AutowareState& msg);

  // Publishers
  rclcpp::Publisher<Engage>::SharedPtr engage_publisher;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pos_publisher;

  // Subscribers
  rclcpp::Subscription<AutowareState>::SharedPtr state_subscriber;

  // Internal states
  int step_counter_;
};

#endif // SELF_DRIVING_CONTROLLER__SELF_DRIVING_CONTROLLER_HPP_