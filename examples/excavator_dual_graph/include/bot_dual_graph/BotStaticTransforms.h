#ifndef BotStaticTransforms_H
#define BotStaticTransforms_H
// Workspace
#include "graph_msf_ros/extrinsics/StaticTransformsTf.h"

namespace bot_est {

class BotStaticTransforms : public graph_msf::StaticTransformsTf {
 public:
  BotStaticTransforms(std::shared_ptr<ros::NodeHandle> privateNodePtr);

  // Setters
  void setMapFrame(const std::string& s) { mapFrame_ = s; }
  void setOdomFrame(const std::string& s) { odomFrame_ = s; }
  void setLidarFrame(const std::string& s) { lidarFrame_ = s; }
  void setBaseFootprintFrame(const std::string& s) { baseFootprintFrame_ = s; }

  // Getters
  const std::string& getMapFrame() { return mapFrame_; }
  const std::string& getOdomFrame() { return odomFrame_; }
  const std::string& getLidarFrame() { return lidarFrame_; }
  const std::string& getBaseFootprintFrame() { return baseFootprintFrame_; }

 private:
  void findTransformations() override;

  // Members
  std::string mapFrame_;
  std::string odomFrame_;
  std::string lidarFrame_;
  std::string baseFootprintFrame_;
};
}  // namespace bot_est
#endif  // end AsopStaticTransforms_H
