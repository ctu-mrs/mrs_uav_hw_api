#define VERSION "1.0.3.0"

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_uav_hw_api/api.h>

#include <pluginlib/class_loader.h>

namespace mrs_uav_hw_api
{

class HwApiManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
};

void HwApiManager::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();
}

}  // namespace mrs_uav_hw_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_hw_api::HwApiManager, nodelet::Nodelet)
