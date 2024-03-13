// Implementation
#include "bot_dual_graph/BotStaticTransforms.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Workspace
#include "graph_msf_ros/util/conversions.h"

namespace bot_est {

BotStaticTransforms::BotStaticTransforms(
    std::shared_ptr<ros::NodeHandle> privateNodePtr)
    : graph_msf::StaticTransformsTf(*privateNodePtr) {
  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START
            << " Initializing static transforms..." << COLOR_END << std::endl;
}

void BotStaticTransforms::findTransformations() {
  // Print to console --------------------------
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END
            << " Looking up transforms in TF-tree.";
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END
            << " Transforms between the following frames are required:"
            << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " "
            << lidarFrame_ << ", " << imuFrame_ << ", " << baseFootprintFrame_
            << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END
            << " Waiting for up to 100 seconds until they arrive..."
            << std::endl;

  // Temporary variable
  static tf::StampedTransform transform;

  // Look up transforms ----------------------------
  //   the tfToMatrix4 would init the map of string pairs of rv_T_frame1_frame2
  // Sleep before subscribing, otherwise sometimes dying in the beginning of
  // rosbag
  ros::Rate rosRate(10);
  rosRate.sleep();

  // Imu to Base Footprint ---
  listener_.waitForTransform(imuFrame_, baseFootprintFrame_, ros::Time(0),
                             ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, baseFootprintFrame_, ros::Time(0),
                            transform);

  // Imu to LiDAR Link ---
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END
            << " Waiting for transform for 10 seconds.";
  listener_.waitForTransform(imuFrame_, lidarFrame_, ros::Time(0),
                             ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, lidarFrame_, ros::Time(0), transform);
  // I_Lidar
  graph_msf::tfToMatrix4(tf::Transform(transform),
                         lv_T_frame1_frame2(imuFrame_, lidarFrame_));
  // Lidar_I is the inverse transformation
  lv_T_frame1_frame2(lidarFrame_, imuFrame_) =
      rv_T_frame1_frame2(imuFrame_, lidarFrame_).inverse();

  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START
            << " Transforms looked up successfully." << COLOR_END << std::endl;
}

}  // namespace bot_est
