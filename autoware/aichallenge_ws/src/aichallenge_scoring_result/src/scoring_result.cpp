#include "aichallenge_scoring_msgs/msg/score.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"

#include <cstdio>
#include <iostream>
#include <fstream>

class ScoringResult : public rclcpp::Node
{
public:
  ScoringResult() : Node("score_node") {
    // Subscribers
    score_subscriber = this->create_subscription<aichallenge_scoring_msgs::msg::Score>(
      "/aichallenge/score", 1, std::bind(&ScoringResult::scoreCallback, this, std::placeholders::_1));
    collision_subscriber = this->create_subscription<geometry_msgs::msg::Point>(
      "/aichallenge/collision", 1, std::bind(&ScoringResult::collisionCallback, this, std::placeholders::_1));
  }

private:

  void scoreCallback(const aichallenge_scoring_msgs::msg::Score& msg) {
    if (has_completed_scoring)
      return;
    
    const auto is_completed_scoring = msg.is_outside_lane || msg.is_timeout || msg.has_exceeded_speed_limit || msg.has_finished_task3;
    if (!is_completed_scoring)
      return;

    std::cout << "Scoring completed." << std::endl;
    writeResultJson(msg);

    has_completed_scoring = true;
  }

  void collisionCallback(const geometry_msgs::msg::Point& msg) {
    std::cout << "Collision occured." << std::endl;

    has_collided = true;
  }

  float calculateDistanceScore(const aichallenge_scoring_msgs::msg::Score& score_msg) {
    if (score_msg.has_exceeded_speed_limit)
      return 0.0f;

    auto penalty_ratio = 1.0f;

    if (!score_msg.has_finished_task1)
      penalty_ratio -= 0.05f;

    // Add penalty if the vehicle goes out of lane or collides after it runs course totally.
    if (score_msg.is_distance_score_maxed_out && (score_msg.is_outside_lane || has_collided))
      penalty_ratio -= 0.05f;

    return score_msg.distance_score * penalty_ratio;
  }

  void writeResultJson(const aichallenge_scoring_msgs::msg::Score& score_msg) {
    std::ofstream ofs("/aichallenge/result.json");
    ofs << "{" << std::endl;
    ofs << "  \"rawDistanceScore\": " << score_msg.distance_score << "," << std::endl;
    ofs << "  \"distanceScore\": " << calculateDistanceScore(score_msg) << "," << std::endl;
    ofs << "  \"task3Duration\": " << score_msg.task3_duration << "," << std::endl;
    ofs << "  \"isOutsideLane\": " << score_msg.is_outside_lane << "," << std::endl;
    ofs << "  \"isStopped\": " << score_msg.is_stopped << "," << std::endl;
    ofs << "  \"hasFinishedTask1\": " << score_msg.has_finished_task1 << "," << std::endl;
    ofs << "  \"hasFinishedTask2\": " << score_msg.has_finished_task2 << "," << std::endl;
    ofs << "  \"hasFinishedTask3\": " << score_msg.has_finished_task3 << std::endl;
    ofs << "}" << std::endl;
    ofs.close();
  }

  // Subscribers
  rclcpp::Subscription<aichallenge_scoring_msgs::msg::Score>::SharedPtr score_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr collision_subscriber;

  rclcpp::TimerBase::SharedPtr timer;

  // AWSIM
  bool has_collided = false;

  // Internal states
  bool has_completed_scoring = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScoringResult>());
  rclcpp::shutdown();
  return 0;
}
