// ROS
#include <ros/ros.h>
// Local packages
#include "bot_dual_graph/BotEstimator.h"

// Main node entry point
int main(int argc, char** argv) {
  // ROS related
  ros::init(argc, argv, "bot_estimator");
  std::shared_ptr<ros::NodeHandle> privateNodePtr =
      std::make_shared<ros::NodeHandle>("~");
  /// Do multi-threaded spinner
  ros::MultiThreadedSpinner spinner(4);

  // Create Instance
  bot_est::BotEstimator botEstimate(privateNodePtr);
  spinner.spin();

  return 0;
}