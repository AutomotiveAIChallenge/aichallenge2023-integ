// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "initialpose_publisher.hpp"

#include <memory>

InitialposePublisher::InitialposePublisher() : Node("initialpose_publisher")
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_cli(cli_initialize_, group_cli_);
  adaptor.init_sub(sub_state_, [this](const State::Message::ConstSharedPtr msg) { state_ = *msg; });

  const auto period = rclcpp::Rate(1.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });

  state_.stamp = now();
  state_.state = State::Message::UNKNOWN;

  initialpose_.pose.pose.position.x = declare_parameter<double>("initialpose.position.x", 0.0);
  initialpose_.pose.pose.position.y = declare_parameter<double>("initialpose.position.y", 0.0);
  initialpose_.pose.pose.position.z = declare_parameter<double>("initialpose.position.z", 0.0);
  initialpose_.pose.pose.orientation.x = declare_parameter<double>("initialpose.orientation.x", 0.0);
  initialpose_.pose.pose.orientation.y = declare_parameter<double>("initialpose.orientation.y", 0.0);
  initialpose_.pose.pose.orientation.z = declare_parameter<double>("initialpose.orientation.z", 0.0);
  initialpose_.pose.pose.orientation.w = declare_parameter<double>("initialpose.orientation.w", 1.0);
}

void InitialposePublisher::on_timer()
{
  timer_->cancel();
 
  if (state_.state == State::Message::UNINITIALIZED) {
    try {
      auto initialpose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

      initialpose_.header.frame_id = "map";
      initialpose_.header.stamp = now();

      initialpose_publisher->publish(initialpose_);
    } catch (const component_interface_utils::ServiceException & error) {
    }
  }
  timer_->reset();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<InitialposePublisher>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
