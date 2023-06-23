#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_system_msgs/msg/autoware_state.hpp"
#include "autoware_auto_vehicle_msgs/msg/engage.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdio>

class SampleCode : public rclcpp::Node
{
public:
  SampleCode() : Node("sample_code")
  {
    // Publishers
    engage_publisher =
      this->create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("/autoware/engage", 1);
    goal_pos_publisher =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 1);
    initialpose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose3d", 1);

    // Subscribers
    state_subscriber = this->create_subscription<autoware_auto_system_msgs::msg::AutowareState>(
      "/autoware/state", 1, std::bind(&SampleCode::stateCallback, this, std::placeholders::_1));

    sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensing/lidar/top/pointcloud_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&SampleCode::pointsCallback, this, std::placeholders::_1));

    // msg
    goal_info_msg = setUpMsgGoalInfo();
    engage_msg = setUpMsgEngage();

    // other
    stepCounter = 0;
    state = 0;
  }

private:
  void pointsCallback(const sensor_msgs::msg::PointCloud2& msg) {
    if (is_pose_initialized_)
      return;
    
    initialpose_publisher->publish(setUpInitialpose());
    is_pose_initialized_ = true;
  }

  void stateCallback(const autoware_auto_system_msgs::msg::AutowareState& msg) {
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
      //std::cout << "Publish the Check Point Info" << std::endl;

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
    msg.pose.position.x = 3724.036376953125;
    msg.pose.position.y = 73754.75;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = -0.9778378972163333;
    msg.pose.orientation.w = 0.2093634322596469;
    return msg;
  }
  
  // setup msg(Initialpose)
  geometry_msgs::msg::PoseWithCovarianceStamped setUpInitialpose()
  {
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = 3827.6742619127863;
    msg.pose.pose.position.y = 73776.76156678918;
    msg.pose.pose.position.z = 19.626154385244764;
    msg.pose.pose.orientation.x = -0.0008439598631592582;
    msg.pose.pose.orientation.y = -0.0010035230204519293;
    msg.pose.pose.orientation.z = -0.9705418231711431;
    msg.pose.pose.orientation.w = 0.24092913926906093;
    return msg;
  }

  // Publishers
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pos_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr check_pos_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_publisher;

  // Subscribers
  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr state_subscriber;
  // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reset_pose_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;


  // msg
  geometry_msgs::msg::PoseStamped goal_info_msg = geometry_msgs::msg::PoseStamped();
  autoware_auto_vehicle_msgs::msg::Engage engage_msg = autoware_auto_vehicle_msgs::msg::Engage();

  // Internal states
  int stepCounter;
  int state;
  bool is_pose_initialized_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleCode>());
  rclcpp::shutdown();
  return 0;
}
