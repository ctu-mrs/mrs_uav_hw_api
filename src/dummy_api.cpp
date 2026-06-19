/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_hw_api/api.h>

#include <nav_msgs/msg/odometry.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>

//}

namespace mrs_uav_hw_api
{

/* class DummyApi //{ */

class DummyApi : public mrs_uav_hw_api::MrsUavHwApi {

public:
  /* ~DummyApi(){}; */

  void initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);

  void destroy();

  // | --------------------- status methods --------------------- |

  mrs_msgs::msg::HwApiStatus       getStatus();
  mrs_msgs::msg::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg);
  bool callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
  bool callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg);
  bool callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgCmd(const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgCmd(const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg);
  bool callbackPositionCmd(const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg);

  void callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void DummyApi::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) {

  common_handlers_ = common_handlers;

  node_  = node;
  clock_ = node->get_clock();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(node_, "DummyHwApi");

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
  }

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "dummy HW API initialized");

  is_initialized_ = true;
}

//}

/* destroy() //{ */

void DummyApi::destroy() {
}

//}

/* getStatus() //{ */

mrs_msgs::msg::HwApiStatus DummyApi::getStatus() {

  mrs_msgs::msg::HwApiStatus diag;

  diag.stamp = clock_->now();

  diag.armed     = false;
  diag.offboard  = false;
  diag.connected = false;

  return diag;
}

//}

/* getCapabilities() //{ */

mrs_msgs::msg::HwApiCapabilities DummyApi::getCapabilities() {

  mrs_msgs::msg::HwApiCapabilities mode;

  mode.api_name = "DummyApi";
  mode.stamp    = clock_->now();

  return mode;
}

//}

/* callbackActuatorCmd() //{ */

bool DummyApi::callbackActuatorCmd([[maybe_unused]] const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool DummyApi::callbackControlGroupCmd([[maybe_unused]] const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool DummyApi::callbackAttitudeRateCmd([[maybe_unused]] const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackAttitudeCmd() //{ */

bool DummyApi::callbackAttitudeCmd([[maybe_unused]] const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool DummyApi::callbackAccelerationHdgRateCmd([[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool DummyApi::callbackAccelerationHdgCmd([[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool DummyApi::callbackVelocityHdgRateCmd([[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool DummyApi::callbackVelocityHdgCmd([[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool DummyApi::callbackPositionCmd([[maybe_unused]] const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) {

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void DummyApi::callbackTrackerCmd([[maybe_unused]] const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {
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

} // namespace mrs_uav_hw_api

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_hw_api::DummyApi, mrs_uav_hw_api::MrsUavHwApi)
