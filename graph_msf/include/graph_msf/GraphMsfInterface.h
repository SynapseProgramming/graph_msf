/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF__INTERFACE_H
#define GRAPH_MSF__INTERFACE_H

#include <Eigen/Eigen>
#include <thread>

// Workspace
#include "StaticTransforms.h"
#include "graph_msf/config/GraphConfig.h"
#include "graph_msf/measurements/BinaryMeasurement6D.h"
#include "graph_msf/measurements/UnaryMeasurement1D.h"
#include "graph_msf/measurements/UnaryMeasurement3D.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"

namespace graph_msf {

class GraphMsf;

class GraphMsfInterface {
 public:
  GraphMsfInterface();

 protected:
  // Setup
  bool setup_();

  // Required for initialization
/**
 * @brief Initializes the yaw and position of the graph with respect to two frames.
 * 
 * @param yaw_W_frame1 The yaw angle in the world frame of frame1.
 * @param frame1 The name of frame1.
 * @param t_W_frame2 The translation vector from the world frame to frame2.
 * @param frame2 The name of frame2.
 * @return True if the initialization is successful, false otherwise.
 */
  bool initYawAndPosition_(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& t_W_frame2,
                           const std::string& frame2);
/**
 * @brief Initializes the yaw and position of the frame.
 *
 * This function takes in a 4x4 transformation matrix T_W_frame and a frame name, and initializes the yaw and position of the frame.
 *
 * @param T_W_frame The 4x4 transformation matrix representing the frame's pose in the world frame.
 * @param frameName The name of the frame.
 * @return True if the initialization is successful, false otherwise.
 */
  bool initYawAndPosition_(const Eigen::Matrix4d& T_W_frame, const std::string& frameName);
/**
 * @brief Checks if the yaw and position have been initialized.
 * 
 * @return True if the yaw and position have been initialized, false otherwise.
 */
  bool areYawAndPositionInited_();

  // Graph Maniupulation

/**
 * @brief Activates the fallback graph.
 * 
 * This function activates the fallback graph, which is used as a backup when the primary graph is not available.
 */
  void activateFallbackGraph();

  // Write measurements
/**
 * @brief Adds an IMU measurement and publishes the state.
 * 
 * This function takes in linear acceleration, angular velocity, and IMU time as input parameters.
 * It adds the IMU measurement to the graph and publishes the updated state.
 * 
 * @param linearAcc The linear acceleration measurement.
 * @param angularVel The angular velocity measurement.
 * @param imuTimeK The IMU time.
 */
  void addImuMeasurementAndPublishState_(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK);
/**
 * @brief Adds an odometry measurement to the graph.
 *
 * This function adds an odometry measurement represented by a BinaryMeasurement6D object to the graph.
 *
 * @param delta The odometry measurement to be added.
 */
  void addOdometryMeasurement_(const BinaryMeasurement6D& delta);
/**
 * @brief Adds a unary pose measurement to the graph.
 *
 * This function adds a unary pose measurement to the graph. The measurement is represented by a UnaryMeasurement6D object.
 *
 * @param unary The unary pose measurement to be added.
 */
  void addUnaryPoseMeasurement_(const UnaryMeasurement6D& unary);
/**
 * @brief Adds a dual odometry measurement to the graph.
 * 
 * This function adds a dual odometry measurement to the graph. It takes two unary odometry measurements,
 * odometryKm1 and odometryK, representing the odometry measurements at time step k-1 and k respectively.
 * The poseBetweenNoise parameter is a 6x1 matrix representing the noise in the pose between the two odometry measurements.
 * 
 * @param odometryKm1 The unary odometry measurement at time step k-1.
 * @param odometryK The unary odometry measurement at time step k.
 * @param poseBetweenNoise The noise in the pose between the two odometry measurements.
 */
  void addDualOdometryMeasurement_(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                   const Eigen::Matrix<double, 6, 1>& poseBetweenNoise);
/**
 * @brief Adds a dual GNSS position measurement to the graph.
 *
 * This function adds a dual GNSS position measurement to the graph. It takes the world frame position measurement
 * `W_t_W_frame`, the last known position `lastPosition`, and the estimated covariance of the XYZ coordinates
 * `estCovarianceXYZ` as input parameters.
 *
 * @param W_t_W_frame The world frame position measurement.
 * @param lastPosition The last known position.
 * @param estCovarianceXYZ The estimated covariance of the XYZ coordinates.
 */
  void addDualGnssPositionMeasurement_(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& lastPosition,
                                       const Eigen::Vector3d& estCovarianceXYZ);
/**
 * @brief Adds a GNSS position measurement to the graph.
 *
 * This function adds a GNSS position measurement to the graph. The measurement is represented by a UnaryMeasurement3D object.
 *
 * @param W_t_W_frame The GNSS position measurement represented by a UnaryMeasurement3D object.
 */
  void addGnssPositionMeasurement_(const UnaryMeasurement3D& W_t_W_frame);
  void addGnssHeadingMeasurement_(const UnaryMeasurement1D& yaw_W_frame);

  // Publish
/**
 * @brief Publishes the state of the graph MSF.
 *
 * This function is responsible for publishing the state of the graph MSF, including the IMU time, 
 * transformation matrices, and linear and angular velocities.
 *
 * @param imuTimeK The IMU time.
 * @param T_W_O The transformation matrix from the world frame to the odometry frame.
 * @param T_O_Ik The transformation matrix from the odometry frame to the IMU frame at time k.
 * @param I_v_W_I The linear velocity of the IMU frame in the world frame.
 * @param I_w_W_I The angular velocity of the IMU frame in the world frame.
 */
  virtual void publishState_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                             const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I) = 0;

  // Getters
  bool isInNormalOperation() const;

  // Member Variables
  /// GraphMsf
  std::shared_ptr<GraphMsf> graphMsfPtr_ = NULL;
  /// Graph Configuration
  std::shared_ptr<GraphConfig> graphConfigPtr_ = NULL;
  std::shared_ptr<StaticTransforms> staticTransformsPtr_ = NULL;

  /// Verbosity
  int verboseLevel_ = 0;

 private:
  // Threads
  std::thread publishStateThread_;

  // Functions
/**
 * @brief Publishes the state and measures the time.
 *
 * This function publishes the state and measures the time using the provided parameters.
 *
 * @param imuTimeK The IMU time.
 * @param T_W_O The transformation matrix from world frame to odometry frame.
 * @param T_O_Ik The transformation matrix from odometry frame to IMU frame.
 * @param I_v_W_I The linear velocity of the IMU in the world frame.
 * @param I_w_W_I The angular velocity of the IMU in the world frame.
 */
  void publishStateAndMeasureTime_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                                   const Eigen::Vector3d& I_v_W_I, const Eigen::Vector3d& I_w_W_I);
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_INTERFACE_H
