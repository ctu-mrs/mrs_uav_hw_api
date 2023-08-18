/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_hw_api/api.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>

//}

namespace mrs_uav_hw_api
{

/* class DummyApi //{ */

class DummyApi : public mrs_uav_hw_api::MrsUavHwApi {

public:
  /* ~DummyApi(){}; */

  void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);

  // | --------------------- status methods --------------------- |

  mrs_msgs::HwApiStatus       getStatus();
  mrs_msgs::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const mrs_msgs::HwApiActuatorCmd::ConstPtr msg);
  bool callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg);
  bool callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg);
  bool callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg);
  bool callbackAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg);
  bool callbackAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg);
  bool callbackVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg);
  bool callbackVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg);
  bool callbackPositionCmd(const mrs_msgs::HwApiPositionCmd::ConstPtr msg);

  void callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void DummyApi::initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh);

  common_handlers_ = common_handlers;

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MrsUavHwApi");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsUavHwDummyApi]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MrsUavHwDummyApi]: initialized");

  is_initialized_ = true;
}

//}

/* getStatus() //{ */

mrs_msgs::HwApiStatus DummyApi::getStatus() {

  mrs_msgs::HwApiStatus diag;

  diag.stamp = ros::Time::now();

  diag.armed     = false;
  diag.offboard  = false;
  diag.connected = false;

  return diag;
}

//}

/* getCapabilities() //{ */

mrs_msgs::HwApiCapabilities DummyApi::getCapabilities() {

  mrs_msgs::HwApiCapabilities mode;

  mode.api_name = "DummyApi";
  mode.stamp    = ros::Time::now();

  return mode;
}

//}

/* callbackActuatorCmd() //{ */

bool DummyApi::callbackActuatorCmd([[maybe_unused]] const mrs_msgs::HwApiActuatorCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool DummyApi::callbackControlGroupCmd([[maybe_unused]] const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool DummyApi::callbackAttitudeRateCmd([[maybe_unused]] const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackAttitudeCmd() //{ */

bool DummyApi::callbackAttitudeCmd([[maybe_unused]] const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool DummyApi::callbackAccelerationHdgRateCmd([[maybe_unused]] const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool DummyApi::callbackAccelerationHdgCmd([[maybe_unused]] const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool DummyApi::callbackVelocityHdgRateCmd([[maybe_unused]] const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool DummyApi::callbackVelocityHdgCmd([[maybe_unused]] const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool DummyApi::callbackPositionCmd([[maybe_unused]] const mrs_msgs::HwApiPositionCmd::ConstPtr msg) {

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void DummyApi::callbackTrackerCmd([[maybe_unused]] const mrs_msgs::TrackerCommand::ConstPtr msg) {
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> DummyApi::callbackArming([[maybe_unused]] const bool &request) {

  return {false, "Dummy interface does not allow to arm."};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> DummyApi::callbackOffboard(void) {

  return {false, "Dummy interface does not allow to switch to offboard."};
}

//}

}  // namespace mrs_uav_hw_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_hw_api::DummyApi, mrs_uav_hw_api::MrsUavHwApi)
