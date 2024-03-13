/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf_ros/GraphMsfRos.h"

// ROS
#include <tf/tf.h>

// Workspace
#include "graph_msf_ros/util/conversions.h"

namespace graph_msf {

/**
 * @brief Adds a pose to the given path message.
 * 
 * @param pathPtr The pointer to the path message.
 * @param frameName The name of the frame.
 * @param stamp The timestamp of the pose.
 * @param t The translation vector of the pose.
 * @param maxBufferLength The maximum buffer length of the path message.
 */
void GraphMsfRos::addToPathMsg(nav_msgs::PathPtr pathPtr, const std::string& frameName, const ros::Time& stamp, const Eigen::Vector3d& t,
                               const int maxBufferLength) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frameName;
  pose.header.stamp = stamp;
  pose.pose.position.x = t(0);
  pose.pose.position.y = t(1);
  pose.pose.position.z = t(2);
  pathPtr->header.frame_id = frameName;
  pathPtr->header.stamp = stamp;
  pathPtr->poses.push_back(pose);
  if (pathPtr->poses.size() > maxBufferLength) {
    pathPtr->poses.erase(pathPtr->poses.begin());
  }
}

/**
 * @brief Adds odometry information to the given Odometry message.
 * 
 * This function adds odometry information to the provided Odometry message. It takes the fixed frame, moving frame,
 * timestamp, transformation matrix, linear velocity in the fixed frame, and angular velocity in the fixed frame as inputs.
 * 
 * @param msgPtr A pointer to the Odometry message to which the odometry information will be added.
 * @param fixedFrame The name of the fixed frame.
 * @param movingFrame The name of the moving frame.
 * @param stamp The timestamp of the odometry information.
 * @param T The transformation matrix representing the pose change between the fixed frame and the moving frame.
 * @param W_v_W_F The linear velocity of the moving frame in the fixed frame.
 * @param W_w_W_F The angular velocity of the moving frame in the fixed frame.
 */
void GraphMsfRos::addToOdometryMsg(nav_msgs::OdometryPtr msgPtr, const std::string& fixedFrame, const std::string& movingFrame,
                                   const ros::Time& stamp, const Eigen::Matrix4d& T, const Eigen::Vector3d& W_v_W_F,
                                   const Eigen::Vector3d& W_w_W_F) {
  msgPtr->header.frame_id = fixedFrame;
  msgPtr->child_frame_id = movingFrame;
  msgPtr->header.stamp = stamp;
  tf::poseTFToMsg(matrix4ToTf(T), msgPtr->pose.pose);
  msgPtr->twist.twist.linear.x = W_v_W_F(0);
  msgPtr->twist.twist.linear.y = W_v_W_F(1);
  msgPtr->twist.twist.linear.z = W_v_W_F(2);
  msgPtr->twist.twist.angular.x = W_w_W_F(0);
  msgPtr->twist.twist.angular.y = W_w_W_F(1);
  msgPtr->twist.twist.angular.z = W_w_W_F(2);
}

long GraphMsfRos::secondsSinceStart_() {
  return std::chrono::duration_cast<std::chrono::seconds>(currentTime_ - startTime_).count();
}

}  // namespace graph_msf