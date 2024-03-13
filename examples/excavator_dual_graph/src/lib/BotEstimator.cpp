#include <bot_dual_graph/BotEstimator.h>

namespace bot_est {

template <typename T>
void printKey(const std::string& key, T value) {
  std::cout << YELLOW_START << "BotEstimator " << COLOR_END << key
            << "  set to: " << value << std::endl;
}
template <>
void printKey(const std::string& key, std::vector<double> vector) {
  std::cout << YELLOW_START << "BotEstimator " << COLOR_END << key
            << " set to: ";
  for (const auto& element : vector) {
    std::cout << element << ",";
  }
  std::cout << std::endl;
}

// Simon Kerscher: Implementation of Templating
template <typename T>
T tryGetParam(const std::string& key, const ros::NodeHandle& privateNode) {
  T value;
  if (privateNode.getParam(key, value)) {
    printKey(key, value);
    return value;
  } else {
    throw std::runtime_error("BotEstimator - " + key + " not specified.");
  }
}

BotEstimator::BotEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr)
    : privateNode_(*privateNodePtr) {
  graphConfigPtr_ = std::make_shared<graph_msf::GraphConfig>();
  staticTransformsPtr_ = std::make_shared<BotStaticTransforms>(privateNodePtr);

  // Get ROS params and set extrinsics
  readParams_(privateNode_);
  staticTransformsPtr_->findTransformations();

}

void BotEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

  if (!graphConfigPtr_) {
    throw std::runtime_error(
        "BotEstimator: graphConfigPtr must be initialized.");
  }

  // Configuration
  /// Using Gnss
  graphConfigPtr_->usingGnssFlag =
      tryGetParam<bool>("launch/usingGnss", privateNode);

  // Set frames
  /// Map
  std::string frame =
      tryGetParam<std::string>("extrinsics/mapFrame", privateNode);
  dynamic_cast<BotStaticTransforms*>(staticTransformsPtr_.get())
      ->setMapFrame(frame);
  /// Odom
  std::cout<<"ran\n";
  frame = tryGetParam<std::string>("extrinsics/odomFrame", privateNode);
  dynamic_cast<BotStaticTransforms*>(staticTransformsPtr_.get())
      ->setOdomFrame(frame);
  /// base_footprint
  frame = tryGetParam<std::string>("extrinsics/baseFootprintFrame",
  privateNode);
  dynamic_cast<BotStaticTransforms*>(staticTransformsPtr_.get())
      ->setBaseFootprintFrame(frame);
  /// IMU
  //// Cabin IMU
  frame = tryGetParam<std::string>("extrinsics/imuFrame", privateNode);
  staticTransformsPtr_->setImuFrame(frame);
  /// LiDAR frame
  frame = tryGetParam<std::string>("extrinsics/lidarFrame", privateNode);
  dynamic_cast<BotStaticTransforms*>(staticTransformsPtr_.get())
      ->setLidarFrame(frame);
}

void BotEstimator::publishState_(const double imuTimeK,
                                 const Eigen::Matrix4d& T_W_O,
                                 const Eigen::Matrix4d& T_O_Ik,
                                 const Eigen::Vector3d& Ic_v_W_Ic,
                                 const Eigen::Vector3d& I_w_W_I) {
  return;
}

void BotEstimator::initializePublishers_(
    std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  return;
}

void BotEstimator::initializeSubscribers_(
    std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  return;
}

}  // namespace bot_est