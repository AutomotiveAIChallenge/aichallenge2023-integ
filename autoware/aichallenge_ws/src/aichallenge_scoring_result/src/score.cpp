#include "aichallenge_scoring_msgs/msg/score.hpp"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cstdio>
#include <iostream>
#include <fstream>

class Score : public rclcpp::Node
{
public:
  Score() : system_clock(RCL_ROS_TIME), Node("score_node")
  {
    // subscriber
    score_subscriber = this->create_subscription<aichallenge_scoring_msgs::msg::Score>(
      "/aichallenge/score", 1, std::bind(&Score::scoreCallback, this, std::placeholders::_1));
  }

private:

  void scoreCallback(const aichallenge_scoring_msgs::msg::Score& msg)
  {
    if (has_published_result)
      return;
      
    std::cout << "Scoring completed." << std::endl;
    writeResultJson(msg);

    has_published_result = true;
  }

  void writeResultJson(const aichallenge_scoring_msgs::msg::Score& score_msg) {
    std::ofstream ofs("/result.json");
    ofs << "{" << std::endl;
    ofs << "  \"rawDistanceScore\": " << score_msg.distance_score << "," << std::endl;
    ofs << "  \"distanceScore\": " << score_msg.distance_score << "," << std::endl;
    ofs << "  \"task3Duration\": " << score_msg.task3_duration << "," << std::endl;
    ofs << "  \"isOutsideLane\": " << score_msg.is_outside_lane << "," << std::endl;
    ofs << "  \"isStopped\": " << score_msg.is_stopped << "," << std::endl;
    ofs << "  \"hasFinishedTask1\": " << score_msg.has_finished_task1 << "," << std::endl;
    ofs << "  \"hasFinishedTask2\": " << score_msg.has_finished_task2 << "," << std::endl;
    ofs << "  \"hasFinishedTask3\": " << score_msg.has_finished_task3 << std::endl;
    ofs << "}" << std::endl;
    ofs.close();
  }

  // subscriber
  rclcpp::Subscription<aichallenge_scoring_msgs::msg::Score>::SharedPtr score_subscriber;

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Clock system_clock;
  double start_sec = 0;
  bool is_start_sec_initialized = false;

  // other
  int start_time = 0;
  bool has_start_time_set = false;
  int end_time = 0;
  int score_time = 0;
  int check_count = 0;
  bool has_published_result = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Score>());
  rclcpp::shutdown();
  return 0;
}
