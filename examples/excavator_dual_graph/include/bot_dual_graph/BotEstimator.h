#ifndef BOTESTIMATOR_H
#define BOTESTIMATOR_H

// std
#include <chrono>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "graph_msf/measurements/UnaryMeasurement6D.h"
#include "graph_msf_ros/GraphMsfRos.h"

// Workspace
#include "bot_dual_graph/BotStaticTransforms.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"
#include "graph_msf_ros/GraphMsfRos.h"

namespace bot_est {

class BotEstimator : public graph_msf::GraphMsfRos {
 public:
  BotEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr);

 private:
  // Parameter Reading Commodity Function
  void readParams_(const ros::NodeHandle& privateNode);

  // Publish State
  void publishState_(const double imuTimeK, const Eigen::Matrix4d& T_W_O,
                     const Eigen::Matrix4d& T_O_Ik,
                     const Eigen::Vector3d& Ic_v_W_Ic,
                     const Eigen::Vector3d& I_w_W_I) override;

  void initializePublishers_(
      std::shared_ptr<ros::NodeHandle>& privateNodePtr) override;

  void initializeSubscribers_(
      std::shared_ptr<ros::NodeHandle>& privateNodePtr) override;

  // Node
  ros::NodeHandle privateNode_;
};

}  // namespace bot_est
#endif  // end BOTESTIMATOR_H