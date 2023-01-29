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

  void initialize(const ros::NodeHandle& parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers, const std::string& topic_prefix,
                  const std::string& uav_name);

  // | --------------------- status methods --------------------- |

  mrs_msgs::HwApiDiagnostics getDiagnostics();
  mrs_msgs::HwApiMode        getMode();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>& wrp);
  bool callbackControlGroupCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>& wrp);
  bool callbackAttitudeRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp);
  bool callbackAttitudeCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp);
  bool callbackAccelerationCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationCmd>& wrp);
  bool callbackVelocityCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityCmd>& wrp);
  bool callbackPositionCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>& wrp);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool& request);
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

void DummyApi::initialize(const ros::NodeHandle& parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers,
                          [[maybe_unused]] const std::string& topic_prefix, [[maybe_unused]] const std::string& uav_name) {

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

/* getDiagnostics() //{ */

mrs_msgs::HwApiDiagnostics DummyApi::getDiagnostics() {

  mrs_msgs::HwApiDiagnostics diag;

  diag.stamp = ros::Time::now();

  diag.armed     = false;
  diag.offboard  = false;
  diag.connected = false;

  return diag;
}

//}

/* getMode() //{ */

mrs_msgs::HwApiMode DummyApi::getMode() {

  mrs_msgs::HwApiMode mode;

  mode.api_name = "DummyApi";
  mode.stamp    = ros::Time::now();

  return mode;
}

//}

/* callbackActuatorCmd() //{ */

bool DummyApi::callbackActuatorCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>& wrp) {

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool DummyApi::callbackControlGroupCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>& wrp) {

  return false;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool DummyApi::callbackAttitudeRateCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp) {

  return false;
}

//}

/* callbackAttitudeCmd() //{ */

bool DummyApi::callbackAttitudeCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp) {

  return false;
}

//}

/* callbackAccelerationCmd() //{ */

bool DummyApi::callbackAccelerationCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationCmd>& wrp) {

  return false;
}

//}

/* callbackVelocityCmd() //{ */

bool DummyApi::callbackVelocityCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityCmd>& wrp) {

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool DummyApi::callbackPositionCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>& wrp) {

  return false;
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> DummyApi::callbackArming([[maybe_unused]] const bool& request) {

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
