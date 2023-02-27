#define VERSION "1.0.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/publisher_handler.h>

#include <mrs_uav_hw_api/api.h>
#include <mrs_uav_hw_api/publishers.h>
#include <mrs_uav_hw_api/common_handlers.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <geometry_msgs/QuaternionStamped.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Range.h>

#include <std_msgs/Float64.h>
#include <mrs_msgs/Float64Stamped.h>

#include <pluginlib/class_loader.h>

//}

namespace mrs_uav_hw_api
{

/* class HwApiManager //{ */

class HwApiManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::string       _version_;
  std::atomic<bool> is_initialized_ = false;

  // | ----------------------- parameters ----------------------- |

  double _timer_diagnostics_rate_;
  double _timer_mode_rate_;

  std::string _plugin_address_;
  std::string _uav_name_;
  std::string _topic_prefix_;

  // | ----------------------- transformer ---------------------- |

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- plugin loader --------------------- |

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_hw_api::MrsUavHwApi>> plugin_loader_;
  boost::shared_ptr<mrs_uav_hw_api::MrsUavHwApi>                       hw_api_;

  // | --------------------- common handlers -------------------- |

  // contains handlers that are shared with trackers and controllers
  // safety area, tf transformer, scope timer logger, and bumper
  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>            sh_actuator_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>        sh_control_group_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>        sh_attitude_rate_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>            sh_attitude_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgRateCmd> sh_acceleration_hdg_rate_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgCmd>     sh_acceleration_hdg_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgRateCmd>     sh_velocity_hdg_rate_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgCmd>         sh_velocity_hdg_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>            sh_position_cmd_;

  void callbackActuatorCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>& wrp);
  void callbackControlGroupCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>& wrp);
  void callbackAttitudeRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp);
  void callbackAttitudeCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp);
  void callbackAccelerationHdgRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgRateCmd>& wrp);
  void callbackAccelerationHdgCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgCmd>& wrp);
  void callbackVelocityHdgRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgRateCmd>& wrp);
  void callbackVelocityHdgCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgCmd>& wrp);
  void callbackPositionCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>& wrp);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::HwApiStatus>       ph_status_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiCapabilities> ph_capabilities_;

  mrs_lib::PublisherHandler<sensor_msgs::NavSatFix>    ph_gnss_;
  mrs_lib::PublisherHandler<sensor_msgs::NavSatStatus> ph_gnss_status_;
  mrs_lib::PublisherHandler<sensor_msgs::Imu>          ph_imu_;
  mrs_lib::PublisherHandler<sensor_msgs::Range>        ph_distance_sensor_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAltitude>   ph_altitude_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>  ph_mag_heading_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiRcChannels> ph_rc_channels_;
  mrs_lib::PublisherHandler<sensor_msgs::BatteryState> ph_battery_state_;

  mrs_lib::PublisherHandler<geometry_msgs::PointStamped>      ph_position_;
  mrs_lib::PublisherHandler<geometry_msgs::Vector3Stamped>    ph_velocity_;
  mrs_lib::PublisherHandler<geometry_msgs::QuaternionStamped> ph_orientation_;
  mrs_lib::PublisherHandler<geometry_msgs::Vector3Stamped>    ph_angular_velocity_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>               ph_odometry_;

  void publishGNSS(const sensor_msgs::NavSatFix& msg);
  void publishGNSSStatus(const sensor_msgs::NavSatStatus& msg);
  void publishOdometry(const nav_msgs::Odometry& msg);
  void publishIMU(const sensor_msgs::Imu& msg);
  void publishDistanceSensor(const sensor_msgs::Range& msg);
  void publishAltitude(const mrs_msgs::HwApiAltitude& msg);
  void publishMagnetometerHeading(const mrs_msgs::Float64Stamped& msg);
  void publishStatus(const mrs_msgs::HwApiStatus& msg);
  void publishRcChannels(const mrs_msgs::HwApiRcChannels& msg);
  void publishOrientation(const geometry_msgs::QuaternionStamped& msg);
  void publishPosition(const geometry_msgs::PointStamped& msg);
  void publishVelocity(const geometry_msgs::Vector3Stamped& msg);
  void publishAngularVelocity(const geometry_msgs::Vector3Stamped& msg);
  void publishBatteryState(const sensor_msgs::BatteryState& msg);

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_diagnostics_;
  ros::Timer timer_mode_;

  void timerDiagnostics(const ros::TimerEvent& event);
  void timerMode(const ros::TimerEvent& event);

  // | --------------------- service servers -------------------- |

  ros::ServiceServer ss_arming_;
  ros::ServiceServer ss_offboard_;

  bool callbackArming(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackOffboard(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // | ----------------------- publishers ----------------------- |
};

//}

/* onInit() //{ */

void HwApiManager::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  // | ----------------------- load params ---------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "HwApiManager");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[HwApiManager]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("hw_interface_plugin", _plugin_address_);
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("topic_prefix", _topic_prefix_);
  param_loader.loadParam("timers/diagnostics/rate", _timer_diagnostics_rate_);
  param_loader.loadParam("timers/mode/rate", _timer_mode_rate_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[HwApiManager]: could not load all parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "HwApiManager");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "HwApiManager";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_actuator_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>(shopts, "actuator_cmd_in", &HwApiManager::callbackActuatorCmd, this);

  sh_control_group_cmd_ =
      mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>(shopts, "control_group_cmd_in", &HwApiManager::callbackControlGroupCmd, this);

  sh_attitude_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>(shopts, "attitude_cmd_in", &HwApiManager::callbackAttitudeCmd, this);

  sh_attitude_rate_cmd_ =
      mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>(shopts, "attitude_rate_cmd_in", &HwApiManager::callbackAttitudeRateCmd, this);

  sh_acceleration_hdg_rate_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgRateCmd>(shopts, "acceleration_hdg_rate_cmd_in",
                                                                                                   &HwApiManager::callbackAccelerationHdgRateCmd, this);

  sh_acceleration_hdg_cmd_ =
      mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgCmd>(shopts, "acceleration_hdg_cmd_in", &HwApiManager::callbackAccelerationHdgCmd, this);

  sh_velocity_hdg_rate_cmd_ =
      mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgRateCmd>(shopts, "velocity_hdg_rate_cmd_in", &HwApiManager::callbackVelocityHdgRateCmd, this);

  sh_velocity_hdg_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgCmd>(shopts, "velocity_hdg_cmd_in", &HwApiManager::callbackVelocityHdgCmd, this);

  sh_position_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>(shopts, "position_cmd_in", &HwApiManager::callbackPositionCmd, this);

  // | ----------------------- publishers ----------------------- |

  ph_capabilities_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiCapabilities>(nh_, "capabilities_out", 1);
  ph_status_       = mrs_lib::PublisherHandler<mrs_msgs::HwApiStatus>(nh_, "status_out", 1);

  ph_gnss_            = mrs_lib::PublisherHandler<sensor_msgs::NavSatFix>(nh_, "gnss_out", 1, false, 50);
  ph_gnss_status_     = mrs_lib::PublisherHandler<sensor_msgs::NavSatStatus>(nh_, "gnss_status_out", 1, false, 10);
  ph_distance_sensor_ = mrs_lib::PublisherHandler<sensor_msgs::Range>(nh_, "distance_sensor_out", 1, false, 250);
  ph_mag_heading_     = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh_, "mag_heading_out", 1, false, 100);
  ph_altitude_        = mrs_lib::PublisherHandler<mrs_msgs::HwApiAltitude>(nh_, "altitude_out", 1, false, 100);
  ph_imu_             = mrs_lib::PublisherHandler<sensor_msgs::Imu>(nh_, "imu_out", 1, false, 500);
  ph_rc_channels_     = mrs_lib::PublisherHandler<mrs_msgs::HwApiRcChannels>(nh_, "rc_channels_out", 1, false, 100);
  ph_battery_state_   = mrs_lib::PublisherHandler<sensor_msgs::BatteryState>(nh_, "battery_state_out", 1, false, 100);

  ph_position_         = mrs_lib::PublisherHandler<geometry_msgs::PointStamped>(nh_, "position_out", 1, false, 250);
  ph_orientation_      = mrs_lib::PublisherHandler<geometry_msgs::QuaternionStamped>(nh_, "orientation_out", 1, false, 250);
  ph_velocity_         = mrs_lib::PublisherHandler<geometry_msgs::Vector3Stamped>(nh_, "velocity_out", 1, false, 250);
  ph_angular_velocity_ = mrs_lib::PublisherHandler<geometry_msgs::Vector3Stamped>(nh_, "angular_velocity_out", 1, false, 500);
  ph_odometry_         = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "odometry_out", 1, false, 250);

  // | --------------------- service servers -------------------- |

  ss_arming_   = nh_.advertiseService("arming_in", &HwApiManager::callbackArming, this);
  ss_offboard_ = nh_.advertiseService("offboard_in", &HwApiManager::callbackOffboard, this);

  // | ------------------------- timers ------------------------- |

  timer_diagnostics_ = nh_.createTimer(ros::Rate(_timer_diagnostics_rate_), &HwApiManager::timerDiagnostics, this);
  timer_mode_        = nh_.createTimer(ros::Rate(_timer_mode_rate_), &HwApiManager::timerMode, this);

  // | ---------------- bind the common handlers ---------------- |

  common_handlers_ = std::make_shared<mrs_uav_hw_api::CommonHandlers_t>();

  common_handlers_->transformer = transformer_;

  common_handlers_->publishers.publishGNSS                = std::bind(&HwApiManager::publishGNSS, this, std::placeholders::_1);
  common_handlers_->publishers.publishGNSSStatus          = std::bind(&HwApiManager::publishGNSSStatus, this, std::placeholders::_1);
  common_handlers_->publishers.publishDistanceSensor      = std::bind(&HwApiManager::publishDistanceSensor, this, std::placeholders::_1);
  common_handlers_->publishers.publishAltitude            = std::bind(&HwApiManager::publishAltitude, this, std::placeholders::_1);
  common_handlers_->publishers.publishIMU                 = std::bind(&HwApiManager::publishIMU, this, std::placeholders::_1);
  common_handlers_->publishers.publishMagnetometerHeading = std::bind(&HwApiManager::publishMagnetometerHeading, this, std::placeholders::_1);
  common_handlers_->publishers.publishStatus              = std::bind(&HwApiManager::publishStatus, this, std::placeholders::_1);
  common_handlers_->publishers.publishRcChannels          = std::bind(&HwApiManager::publishRcChannels, this, std::placeholders::_1);
  common_handlers_->publishers.publishBatteryState        = std::bind(&HwApiManager::publishBatteryState, this, std::placeholders::_1);

  common_handlers_->publishers.publishPosition        = std::bind(&HwApiManager::publishPosition, this, std::placeholders::_1);
  common_handlers_->publishers.publishOrientation     = std::bind(&HwApiManager::publishOrientation, this, std::placeholders::_1);
  common_handlers_->publishers.publishVelocity        = std::bind(&HwApiManager::publishVelocity, this, std::placeholders::_1);
  common_handlers_->publishers.publishAngularVelocity = std::bind(&HwApiManager::publishAngularVelocity, this, std::placeholders::_1);
  common_handlers_->publishers.publishOdometry        = std::bind(&HwApiManager::publishOdometry, this, std::placeholders::_1);

  // | -------------------- load the plugin -------------------- |

  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_hw_api::MrsUavHwApi>>("mrs_uav_hw_api", "mrs_uav_hw_api::MrsUavHwApi");

  try {
    ROS_INFO("[HwApiManager]: loading the plugin '%s'", _plugin_address_.c_str());
    hw_api_ = plugin_loader_->createInstance(_plugin_address_.c_str());
  }
  catch (pluginlib::CreateClassException& ex1) {
    ROS_ERROR("[HwApiManager]: CreateClassException for the plugin '%s'", _plugin_address_.c_str());
    ROS_ERROR("[HwApiManager]: Error: %s", ex1.what());
    ros::shutdown();
  }
  catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR("[HwApiManager]: PluginlibException for the plugin '%s'", _plugin_address_.c_str());
    ROS_ERROR("[HwApiManager]: Error: %s", ex.what());
    ros::shutdown();
  }

  // | ------------------ initialize the plugin ----------------- |

  hw_api_->initialize(nh_, common_handlers_, _topic_prefix_, _uav_name_);

  ROS_INFO("[HwApiManager]: initialized");

  is_initialized_ = true;
}

//}

// | --------------------- topic callbacks -------------------- |

/* callbackActuatorCmd() //{ */

void HwApiManager::callbackActuatorCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackActuatorCmd(wrp);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'control group' command!");
  }
}

//}

/* callbackControlGroupCmd() //{ */

void HwApiManager::callbackControlGroupCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackControlGroupCmd(wrp);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'control group' command!");
  }
}

//}

/* callbackAttitudeRateCmd() //{ */

void HwApiManager::callbackAttitudeRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAttitudeRateCmd(wrp);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'attitude rate' command!");
  }
}

//}

/* callbackAttitudeCmd() //{ */

void HwApiManager::callbackAttitudeCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAttitudeCmd(wrp);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'attitude' command!");
  }
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

void HwApiManager::callbackAccelerationHdgRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgRateCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAccelerationHdgRateCmd(wrp);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'acceleration+hdg rate' command!");
  }
}

//}

/* callbackAccelerationHdgCmd() //{ */

void HwApiManager::callbackAccelerationHdgCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAccelerationHdgCmd(wrp);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'acceleration+hdg' command!");
  }
}

//}

/* callbackVelocityHdgRateCmd() //{ */

void HwApiManager::callbackVelocityHdgRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgRateCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackVelocityHdgRateCmd(wrp);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'velocity+hdg rate' command!");
  }
}

//}

/* callbackVelocityHdgCmd() //{ */

void HwApiManager::callbackVelocityHdgCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackVelocityHdgCmd(wrp);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'velocity+hdg' command!");
  }
}

//}

/* callbackPositionCmd() //{ */

void HwApiManager::callbackPositionCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackPositionCmd(wrp);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'position' command!");
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackArming() //{ */

bool HwApiManager::callbackArming(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  ROS_INFO("[HwApiManager]: %s", req.data ? "arming" : "disarming");

  auto [success, message] = hw_api_->callbackArming(req.data);

  res.success = success;
  res.message = message;

  return true;
}

//}

/* callbackOffboard() //{ */

bool HwApiManager::callbackOffboard([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  ROS_INFO("[HwApiManager]: switching to offboard");

  auto [success, message] = hw_api_->callbackOffboard();

  res.success = success;
  res.message = message;

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerDiagnostics() //{ */

void HwApiManager::timerDiagnostics([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[HwApiManager]: timerDiagnostics() spinning");

  mrs_msgs::HwApiStatus diag = hw_api_->getStatus();

  ph_status_.publish(diag);
}

//}

/* timerMode() //{ */

void HwApiManager::timerMode([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[HwApiManager]: timerCapabilities() spinning");

  mrs_msgs::HwApiCapabilities diag = hw_api_->getCapabilities();

  ph_capabilities_.publish(diag);
}

//}

// | ----------------------- publishers ----------------------- |

/* publishGNSS() //{ */

void HwApiManager::publishGNSS(const sensor_msgs::NavSatFix& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_gnss_.publish(msg);
}

//}

/* publishGNSSStatus() //{ */

void HwApiManager::publishGNSSStatus(const sensor_msgs::NavSatStatus& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_gnss_status_.publish(msg);
}

//}

/* publishIMU() //{ */

void HwApiManager::publishIMU(const sensor_msgs::Imu& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_imu_.publish(msg);
}

//}

/* publishDistanceSensor() //{ */

void HwApiManager::publishDistanceSensor(const sensor_msgs::Range& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_distance_sensor_.publish(msg);
}

//}

/* publishAltitude() //{ */

void HwApiManager::publishAltitude(const mrs_msgs::HwApiAltitude& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_altitude_.publish(msg);
}

//}

/* publishMagnetometerHeading() //{ */

void HwApiManager::publishMagnetometerHeading(const mrs_msgs::Float64Stamped& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_mag_heading_.publish(msg);
}

//}

/* publishStatus() //{ */

void HwApiManager::publishStatus(const mrs_msgs::HwApiStatus& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_status_.publish(msg);
}

//}

/* publishRcChannels() //{ */

void HwApiManager::publishRcChannels(const mrs_msgs::HwApiRcChannels& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_rc_channels_.publish(msg);
}

//}

/* publishPosition() //{ */

void HwApiManager::publishPosition(const geometry_msgs::PointStamped& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_position_.publish(msg);
}

//}

/* publishOrientation() //{ */

void HwApiManager::publishOrientation(const geometry_msgs::QuaternionStamped& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_orientation_.publish(msg);
}

//}

/* publishVelocity() //{ */

void HwApiManager::publishVelocity(const geometry_msgs::Vector3Stamped& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_velocity_.publish(msg);
}

//}

/* publishAngularVelocity() //{ */

void HwApiManager::publishAngularVelocity(const geometry_msgs::Vector3Stamped& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_angular_velocity_.publish(msg);
}

//}

/* publishOdometry() //{ */

void HwApiManager::publishOdometry(const nav_msgs::Odometry& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_odometry_.publish(msg);
}

//}

/* publishBatteryState() //{ */

void HwApiManager::publishBatteryState(const sensor_msgs::BatteryState& msg) {

  if (!is_initialized_) {
    return;
  }

  ph_battery_state_.publish(msg);
}

//}

}  // namespace mrs_uav_hw_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_hw_api::HwApiManager, nodelet::Nodelet)
