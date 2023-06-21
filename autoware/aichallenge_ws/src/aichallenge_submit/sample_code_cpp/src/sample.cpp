#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_system_msgs/msg/autoware_state.hpp"
#include "autoware_auto_vehicle_msgs/msg/engage.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "std_msgs/msg/string.hpp"

#include <cstdio>

class SampleCode : public rclcpp::Node
{
public:
  SampleCode() : Node("sample_code")
  {
    // publisher
    engage_publisher =
      this->create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("/autoware/engage", 1);
    goal_pos_publisher =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 1);
    initialpose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1);

    // subscriber
    state_subscriber = this->create_subscription<autoware_auto_system_msgs::msg::AutowareState>(
      "/autoware/state", 1, std::bind(&SampleCode::stateCallback, this, std::placeholders::_1));
    reset_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose3d", 1, std::bind(&SampleCode::resetPoseCallback, this, std::placeholders::_1));

    // msg
    check_info_msg = setUpMsgCheckInfo();
    goal_info_msg = setUpMsgGoalInfo();
    engage_msg = setUpMsgEngage();

    // other
    stepCounter = 0;
    state = 0;
  }

private:
  void stateCallback(const autoware_auto_system_msgs::msg::AutowareState msg)
  {
    state = msg.state;
    // std::cout << state << std::endl;

    // ego vehicle base
    // Goal Pos Pub --> Check Point Pub --> Engage Pub

    //  Publish the Goal Point Info
    if (state == 2 && stepCounter == 0) {
      stepCounter++;

      // publish
      goal_pos_publisher->publish(goal_info_msg);
      std::cout << "Publish the Goal Point Info" << std::endl;

      // Publish the Check Point Info
    } else if (state == 4 && stepCounter == 1) {
      stepCounter++;

      // publish
      //check_pos_publisher->publish(check_info_msg);
      std::cout << "Publish the Check Point Info" << std::endl;

      // Publish the Engage, Vehicle drive
    } else if (state == 4 && stepCounter == 2) {
      stepCounter++;
      // publish
      std::cout << "Publish the Engage" << std::endl;
      engage_publisher->publish(engage_msg);
    } else if (state == 3) {
      std::cout << "Planning....." << std::endl;
    }
  }


  void resetPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped msg)
  {
    static bool is_active = true;
    if (is_active) {
      is_active = false;
      initialpose_publisher->publish(setUpInitialpose());
      dummy_objects_publisher->publish(autoware_auto_perception_msgs::msg::PredictedObjects());
    }
  }

  // setup msg(engage)
  autoware_auto_vehicle_msgs::msg::Engage setUpMsgEngage()
  {
    auto msg = autoware_auto_vehicle_msgs::msg::Engage();
    msg.engage = true;
    return msg;
  }

  // setup msg(goal info)
  geometry_msgs::msg::PoseStamped setUpMsgGoalInfo()
  {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.frame_id = "map";
    msg.pose.position.x = 3827.680908203125 - 45.5;
    msg.pose.position.y = 73776.7578125 - 23.9;
    msg.pose.position.z = 19.627578735351562;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.9705480378755114;
    msg.pose.orientation.w = -0.2409040617944975;

    return msg;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped setUpInitialpose()
  {
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = 3827.680908203125;
    msg.pose.pose.position.y = 73776.7578125;
    msg.pose.pose.position.z = 19.627578735351562;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.9705480378755114;
    msg.pose.pose.orientation.w = -0.2409040617944975;
    return msg;
  }

  // setup msg(check info)

  geometry_msgs::msg::PoseStamped setUpMsgCheckInfo()
  {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.frame_id = "map";
    msg.pose.position.x = 3820.680908203125;
    msg.pose.position.y = 73776.7578125;
    msg.pose.position.z = 19.627578735351562;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.9705480378755114;
    msg.pose.orientation.w = -0.2409040617944975;
    return msg;
  }

  // publisher
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pos_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr check_pos_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_publisher;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr dummy_objects_publisher;

  // subscriber
  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr state_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reset_pose_subscriber;

  // msg
  geometry_msgs::msg::PoseStamped check_info_msg = geometry_msgs::msg::PoseStamped();
  geometry_msgs::msg::PoseStamped goal_info_msg = geometry_msgs::msg::PoseStamped();
  autoware_auto_vehicle_msgs::msg::Engage engage_msg = autoware_auto_vehicle_msgs::msg::Engage();
  // other
  int stepCounter;
  int state;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleCode>());
  rclcpp::shutdown();
  return 0;
}
