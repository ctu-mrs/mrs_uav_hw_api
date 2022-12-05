#define VERSION "1.0.3.0"

/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>

#include <mrs_uav_hw_api/api.h>

#include <std_srvs/SetBool.h>

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

  std::string _plugin_address_;
  std::string _uav_name_;

  // | ----------------------- transformer ---------------------- |

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- plubin loader --------------------- |

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_hw_api::MrsUavHwApi>> plugin_loader_;
  boost::shared_ptr<mrs_uav_hw_api::MrsUavHwApi>                       hw_api_;

  // | --------------------- common handlers -------------------- |

  // contains handlers that are shared with trackers and controllers
  // safety area, tf transformer, scope timer logger, and bumper
  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd> sh_control_group_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd> sh_attitude_rate_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>     sh_attitude_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiTranslationCmd>  sh_translation_cmd_;

  void callbackControlGroupCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>& wrp);
  void callbackAttitudeRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp);
  void callbackAttitudeCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp);
  void callbackTranslationCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiTranslationCmd>& wrp);

  // | --------------------- service servers -------------------- |

  ros::ServiceServer ss_arming_;
  ros::ServiceServer ss_offboard_;

  bool callbackArming(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool callbackOffboard(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

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
  param_loader.loadParam("timers/diagnostics/rate", _timer_diagnostics_rate_);

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

  sh_control_group_cmd_ =
      mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>(shopts, "control_group_cmd_in", &HwApiManager::callbackControlGroupCmd, this);

  sh_attitude_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>(shopts, "attitude_cmd_in", &HwApiManager::callbackAttitudeCmd, this);


  sh_attitude_rate_cmd_ =
      mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>(shopts, "attitude_rate_cmd_in", &HwApiManager::callbackAttitudeRateCmd, this);

  sh_translation_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiTranslationCmd>(shopts, "translation_cmd_in", &HwApiManager::callbackTranslationCmd, this);

  // | --------------------- service servers -------------------- |

  ss_arming_   = nh_.advertiseService("arming_in", &HwApiManager::callbackArming, this);
  ss_offboard_ = nh_.advertiseService("offboard_in", &HwApiManager::callbackOffboard, this);

  // | ---------------- bind the common handlers ---------------- |

  common_handlers_->transformer = transformer_;

  // | -------------------- load the plugin -------------------- |

  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_hw_api::MrsUavHwApi>>("mrs_uav_hw_api", "mrs_uav_hw_api::HwApiManager");

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

  /* hw_api_->initialize(nh_, common_handlers_); */
}

//}

// | --------------------- topic callbacks -------------------- |

/* callbackControlGroupCmd() //{ */

void HwApiManager::callbackControlGroupCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>& wrp) {

  if (!is_initialized_)
    return;

  auto msg = wrp.getMsg();

  bool result = hw_api_->callbackControlGroupCmd(*msg);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'control group' command!");
  }
}

//}

/* callbackAttitudeRateCmd() //{ */

void HwApiManager::callbackAttitudeRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp) {

  if (!is_initialized_)
    return;

  auto msg = wrp.getMsg();

  bool result = hw_api_->callbackAttitudeRateCmd(*msg);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'control group' command!");
  }
}

//}

/* callbackAttitudeRateCmd() //{ */

void HwApiManager::callbackAttitudeCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp) {

  if (!is_initialized_)
    return;

  auto msg = wrp.getMsg();

  bool result = hw_api_->callbackAttitudeCmd(*msg);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'control group' command!");
  }
}

//}

/* callbackTranslationCmd() //{ */

void HwApiManager::callbackTranslationCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiTranslationCmd>& wrp) {

  if (!is_initialized_)
    return;

  auto msg = wrp.getMsg();

  bool result = hw_api_->callbackTranslationCmd(*msg);

  if (!result) {
    ROS_WARN_THROTTLE(1.0, "[HwApiManager]: the currently loaded HW API does not implement the 'control group' command!");
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackArming() //{ */

bool HwApiManager::callbackArming(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_)
    return false;

  ROS_INFO("[HwApiManager]: %s", req.data ? "arming" : "disarming");

  auto [success, message] = hw_api_->callbackArming(req.data);

  res.success = success;
  res.message = message;

  return true;
}

//}

/* callbackOffboard() //{ */

bool HwApiManager::callbackOffboard(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_)
    return false;

  ROS_INFO("[HwApiManager]: offboard %s", req.data ? "on" : "off");

  auto [success, message] = hw_api_->callbackOffboard(req.data);

  res.success = success;
  res.message = message;

  return true;
}

//}

}  // namespace mrs_uav_hw_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_hw_api::HwApiManager, nodelet::Nodelet)
