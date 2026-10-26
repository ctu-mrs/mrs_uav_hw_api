#define VERSION "2.0.0.0"

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/errorgraph/error_publisher.h>

#include <mrs_uav_hw_api/api.h>
#include <mrs_uav_hw_api/publishers.h>
#include <mrs_uav_hw_api/common_handlers.h>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>

#include <pluginlib/class_loader.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <set>

//}

/* using //{ */

using namespace std::chrono_literals;

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_hw_api
{

/* class HwApiManager //{ */

class HwApiManager : public mrs_lib::Node {

public:
  HwApiManager(rclcpp::NodeOptions options);
  ~HwApiManager();

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  std::string              _version_;
  std::atomic<bool>        is_initialized_ = false;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  void                             initialize();
  void                             shutdown();
  rclcpp::OnShutdownCallbackHandle shutdown_callback_handle_;

  // | ----------------------- parameters ----------------------- |

  double _timer_diagnostics_rate_;
  double _timer_mode_rate_;

  double _pub_gnss_rate_;
  double _pub_gnss_status_rate_;
  double _pub_rtk_rate_;
  double _pub_imu_rate_;
  double _pub_distance_sensor_rate_;
  double _pub_altitude_rate_;
  double _pub_mag_heading_rate_;
  double _pub_mag_magnetic_field_rate_;
  double _pub_rc_channels_rate_;
  double _pub_battery_state_rate_;

  double _pub_position_rate_;
  double _pub_velocity_rate_;
  double _pub_orientation_rate_;
  double _pub_angular_velocity_rate_;
  double _pub_odometry_rate_;
  double _pub_ground_truth_rate_;

  std::string _plugin_address_;
  std::string _uav_name_;
  std::string _body_frame_name_;
  std::string _world_frame_name_;
  std::string _topic_prefix_;

  // | ---------------------- errorgraph --------------------- |

  enum class error_type_t : uint16_t
  {
    version_mismatch,
    parameter_loading,
    not_connected,
  };

  std::unique_ptr<mrs_lib::errorgraph::ErrorPublisher> error_publisher_;

  // | ---------------------- param loader ---------------------- |

  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;

  // | ---------------------- plugin loader --------------------- |

  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_hw_api::MrsUavHwApi>> plugin_loader_;
  std::shared_ptr<mrs_uav_hw_api::MrsUavHwApi>                         hw_api_;

  // | --------------------- common handlers -------------------- |

  // contains handlers that are shared with the HW API plugin
  // publishers, param loader, and the uav name and frame getters
  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiActuatorCmd>            sh_actuator_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiControlGroupCmd>        sh_control_group_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAttitudeRateCmd>        sh_attitude_rate_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAttitudeCmd>            sh_attitude_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAccelerationHdgRateCmd> sh_acceleration_hdg_rate_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAccelerationHdgCmd>     sh_acceleration_hdg_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiVelocityHdgRateCmd>     sh_velocity_hdg_rate_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiVelocityHdgCmd>         sh_velocity_hdg_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiPositionCmd>            sh_position_cmd_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand> sh_tracker_cmd_;

  void callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg);
  void callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
  void callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg);
  void callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg);
  void callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg);
  void callbackAccelerationHdgCmd(const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg);
  void callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg);
  void callbackVelocityHdgCmd(const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg);
  void callbackPositionCmd(const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg);
  void callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiStatus>       ph_status_;
  mrs_lib::PublisherHandler<std_msgs::msg::Empty>             ph_connected_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiCapabilities> ph_capabilities_;

  mrs_lib::PublisherHandler<sensor_msgs::msg::NavSatFix>     ph_gnss_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::GpsInfo>          ph_gnss_status_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::RtkGps>           ph_rtk_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>           ph_imu_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::Range>         ph_distance_sensor_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAltitude>    ph_altitude_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>   ph_mag_heading_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::MagneticField> ph_mag_magnetic_field_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiRcChannels>  ph_rc_channels_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiRcRssi>      ph_rc_rssi_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::BatteryState>  ph_battery_state_;

  mrs_lib::PublisherHandler<geometry_msgs::msg::PointStamped>      ph_position_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>    ph_velocity_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped> ph_orientation_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>    ph_angular_velocity_;
  mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>               ph_odometry_;
  mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>               ph_ground_truth_;

  std::string getUavName(void);
  std::string getBodyFrameName(void);
  std::string getWorldFrameName(void);

  void publishGNSS(const sensor_msgs::msg::NavSatFix &msg);
  void publishGNSSStatus(const mrs_msgs::msg::GpsInfo &msg);
  void publishRTK(const mrs_msgs::msg::RtkGps &msg);
  void publishOdometry(const nav_msgs::msg::Odometry &msg);
  void publishGroundTruth(const nav_msgs::msg::Odometry &msg);
  void publishIMU(const sensor_msgs::msg::Imu &msg);
  void publishDistanceSensor(const sensor_msgs::msg::Range &msg);
  void publishAltitude(const mrs_msgs::msg::HwApiAltitude &msg);
  void publishMagnetometerHeading(const mrs_msgs::msg::Float64Stamped &msg);
  void publishMagneticField(const sensor_msgs::msg::MagneticField &msg);
  void publishStatus(const mrs_msgs::msg::HwApiStatus &msg);
  void publishRcChannels(const mrs_msgs::msg::HwApiRcChannels &msg);
  void publishRcRssi(const mrs_msgs::msg::HwApiRcRssi &msg);
  void publishOrientation(const geometry_msgs::msg::QuaternionStamped &msg);
  void publishPosition(const geometry_msgs::msg::PointStamped &msg);
  void publishVelocity(const geometry_msgs::msg::Vector3Stamped &msg);
  void publishAngularVelocity(const geometry_msgs::msg::Vector3Stamped &msg);
  void publishBatteryState(const sensor_msgs::msg::BatteryState &msg);

  // the inputs and outputs requested by the configs
  mrs_msgs::msg::HwApiCapabilities requested_capabilities_;

  // requested and provided by the plugin, the topics exist only for these
  mrs_msgs::msg::HwApiCapabilities capabilities_;

  // (config key, resolved topic)
  typedef std::vector<std::pair<std::string, std::string>> topic_list_t;

  bool checkConfigKeys(const std::vector<std::string> &config_files);
  bool hasYamlKey(const YAML::Node &yaml, const std::string &path);
  void loadRequestedCapabilities(const std::string &yaml_prefix);
  bool useCapability(const bool requested, bool &capability, const std::string &key, const std::string &topic, topic_list_t &disabled);
  void initCommandSubscribers(const mrs_lib::SubscriberHandlerOptions &shopts, topic_list_t &not_subscribed);
  void initOutputPublishers(topic_list_t &not_published);
  void logDisabledTopics(const topic_list_t &not_subscribed, const topic_list_t &not_published);

  template <typename MessageType>
  void publishOutput(const bool produced, const std::string &key, mrs_lib::PublisherHandler<MessageType> &publisher, const MessageType &msg);

  // | ------------------------- timers ------------------------- |

  std::shared_ptr<TimerType> timer_diagnostics_;
  std::shared_ptr<TimerType> timer_mode_;

  void timerStatus(void);
  void timerMode(void);

  // | --------------------- service servers -------------------- |

  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool> ss_arming_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_offboard_;

  bool callbackArming(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool callbackOffboard(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // | ----------------------- publishers ----------------------- |
};

//}

/* HwApiManager() //{ */

HwApiManager::HwApiManager(rclcpp::NodeOptions options) : mrs_lib::Node("hw_api_manager", options) {

  this->initialize();
}

//}

/* ~HwApiManager() //{ */

HwApiManager::~HwApiManager() {
  rclcpp::contexts::get_global_default_context()->remove_on_shutdown_callback(shutdown_callback_handle_);
}

//}

/* initialize() //{ */

void HwApiManager::initialize() {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  shutdown_callback_handle_ = rclcpp::contexts::get_global_default_context()->add_on_shutdown_callback([this]() { this->shutdown(); });

  error_publisher_ = std::make_unique<mrs_lib::errorgraph::ErrorPublisher>(node_, clock_, "HwApiManager", "main");

  // | ----------------------- load params ---------------------- |

  param_loader_ = std::make_shared<mrs_lib::ParamLoader>(node_);

  std::string custom_config;
  std::string platform_config;
  param_loader_->loadParam("custom_config", custom_config, std::string(""));
  param_loader_->loadParam("platform_config", platform_config, std::string(""));

  std::vector<std::string> default_configs;
  param_loader_->loadParam("configs", default_configs);

  // the first loaded file has the priority
  std::vector<std::string> config_files;

  if (custom_config != "") {
    config_files.push_back(custom_config);
  }

  if (platform_config != "") {
    config_files.push_back(platform_config);
  }

  config_files.insert(config_files.end(), default_configs.begin(), default_configs.end());

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "loading config file '%s'", config_file.c_str());
    if (!param_loader_->addYamlFile(config_file)) {
      RCLCPP_ERROR(node_->get_logger(), "failed to load config file '%s'", config_file.c_str());
      error_publisher_->addOneshotError("failed to load config file '" + config_file + "'");
      error_publisher_->flushAndShutdown();
    }
  }

  if (!checkConfigKeys(config_files)) {
    error_publisher_->flushAndShutdown();
  }

  // params from the launch file are not prefixed
  param_loader_->loadParam("uav_name", _uav_name_);
  param_loader_->loadParam("topic_prefix", _topic_prefix_);

  const std::string yaml_prefix = "mrs_uav_hw_api/";

  param_loader_->loadParam(yaml_prefix + "version", _version_);

  if (_version_ != VERSION) {

    RCLCPP_ERROR(node_->get_logger(), "the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    error_publisher_->addGeneralError(error_type_t::version_mismatch,
                                      "version mismatch: binary version " + std::string(VERSION) + " != config version " + _version_);

    error_publisher_->flushAndShutdown();
  }

  param_loader_->loadParam(yaml_prefix + "hw_interface_plugin", _plugin_address_);
  param_loader_->loadParam(yaml_prefix + "body_frame_name", _body_frame_name_);
  param_loader_->loadParam(yaml_prefix + "world_frame_name", _world_frame_name_);
  param_loader_->loadParam(yaml_prefix + "timers/diagnostics/rate", _timer_diagnostics_rate_);
  param_loader_->loadParam(yaml_prefix + "timers/mode/rate", _timer_mode_rate_);

  param_loader_->loadParam(yaml_prefix + "publish_rate/gnss", _pub_gnss_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/gnss_status", _pub_gnss_status_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/rtk", _pub_rtk_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/imu", _pub_imu_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/distance_sensor", _pub_distance_sensor_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/altitude", _pub_altitude_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/mag_heading", _pub_mag_heading_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/mag_magnetic_field", _pub_mag_magnetic_field_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/rc_channels", _pub_rc_channels_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/battery_state", _pub_battery_state_rate_);

  param_loader_->loadParam(yaml_prefix + "publish_rate/position", _pub_position_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/velocity", _pub_velocity_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/orientation", _pub_orientation_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/angular_velocity", _pub_angular_velocity_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/odometry", _pub_odometry_rate_);
  param_loader_->loadParam(yaml_prefix + "publish_rate/ground_truth", _pub_ground_truth_rate_);

  loadRequestedCapabilities(yaml_prefix);

  if (!param_loader_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    error_publisher_->addGeneralError(error_type_t::parameter_loading, "could not load all parameters");
    error_publisher_->flushAndShutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.node_name                           = "HwApiManager";
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_tracker_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(shopts, "/" + _uav_name_ + "/control_manager/tracker_cmd",
                                                                              &HwApiManager::callbackTrackerCmd, this);

  // | ----------------------- publishers ----------------------- |


  {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node = node_;

    ph_capabilities_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiCapabilities>(opts, "~/capabilities");
  }

  {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node = node_;

    ph_status_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiStatus>(opts, "~/status");
  }

  {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node = node_;

    ph_connected_ = mrs_lib::PublisherHandler<std_msgs::msg::Empty>(opts, "~/connected");
  }

  // | --------------------- service servers -------------------- |

  ss_arming_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(
      node_, "~/arming", std::bind(&HwApiManager::callbackArming, this, std::placeholders::_1, std::placeholders::_2), cbkgrp_ss_);

  ss_offboard_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/offboard", std::bind(&HwApiManager::callbackOffboard, this, std::placeholders::_1, std::placeholders::_2), cbkgrp_ss_);

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions opts;

  opts.node           = node_;
  opts.autostart      = true;
  opts.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&HwApiManager::timerStatus, this);

    timer_diagnostics_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_timer_diagnostics_rate_, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&HwApiManager::timerMode, this);

    timer_mode_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_timer_mode_rate_, clock_), callback_fcn);
  }

  // | ---------------- bind the common handlers ---------------- |

  common_handlers_ = std::make_shared<mrs_uav_hw_api::CommonHandlers_t>();

  common_handlers_->main_param_loader = param_loader_;

  common_handlers_->getUavName        = std::bind(&HwApiManager::getUavName, this);
  common_handlers_->getBodyFrameName  = std::bind(&HwApiManager::getBodyFrameName, this);
  common_handlers_->getWorldFrameName = std::bind(&HwApiManager::getWorldFrameName, this);

  common_handlers_->publishers.publishGNSS                = std::bind(&HwApiManager::publishGNSS, this, std::placeholders::_1);
  common_handlers_->publishers.publishGNSSStatus          = std::bind(&HwApiManager::publishGNSSStatus, this, std::placeholders::_1);
  common_handlers_->publishers.publishRTK                 = std::bind(&HwApiManager::publishRTK, this, std::placeholders::_1);
  common_handlers_->publishers.publishDistanceSensor      = std::bind(&HwApiManager::publishDistanceSensor, this, std::placeholders::_1);
  common_handlers_->publishers.publishAltitude            = std::bind(&HwApiManager::publishAltitude, this, std::placeholders::_1);
  common_handlers_->publishers.publishIMU                 = std::bind(&HwApiManager::publishIMU, this, std::placeholders::_1);
  common_handlers_->publishers.publishMagnetometerHeading = std::bind(&HwApiManager::publishMagnetometerHeading, this, std::placeholders::_1);
  common_handlers_->publishers.publishMagneticField       = std::bind(&HwApiManager::publishMagneticField, this, std::placeholders::_1);
  common_handlers_->publishers.publishStatus              = std::bind(&HwApiManager::publishStatus, this, std::placeholders::_1);
  common_handlers_->publishers.publishRcChannels          = std::bind(&HwApiManager::publishRcChannels, this, std::placeholders::_1);
  common_handlers_->publishers.publishRcRssi              = std::bind(&HwApiManager::publishRcRssi, this, std::placeholders::_1);
  common_handlers_->publishers.publishBatteryState        = std::bind(&HwApiManager::publishBatteryState, this, std::placeholders::_1);

  common_handlers_->publishers.publishPosition        = std::bind(&HwApiManager::publishPosition, this, std::placeholders::_1);
  common_handlers_->publishers.publishOrientation     = std::bind(&HwApiManager::publishOrientation, this, std::placeholders::_1);
  common_handlers_->publishers.publishVelocity        = std::bind(&HwApiManager::publishVelocity, this, std::placeholders::_1);
  common_handlers_->publishers.publishAngularVelocity = std::bind(&HwApiManager::publishAngularVelocity, this, std::placeholders::_1);
  common_handlers_->publishers.publishOdometry        = std::bind(&HwApiManager::publishOdometry, this, std::placeholders::_1);
  common_handlers_->publishers.publishGroundTruth     = std::bind(&HwApiManager::publishGroundTruth, this, std::placeholders::_1);

  // | -------------------- load the plugin -------------------- |

  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_hw_api::MrsUavHwApi>>("mrs_uav_hw_api", "mrs_uav_hw_api::MrsUavHwApi");

  try {
    RCLCPP_INFO(node_->get_logger(), "loading the plugin '%s'", _plugin_address_.c_str());
    hw_api_ = plugin_loader_->createSharedInstance(_plugin_address_.c_str());
  }
  catch (pluginlib::CreateClassException &ex1) {
    RCLCPP_ERROR(node_->get_logger(), "CreateClassException for the plugin '%s'", _plugin_address_.c_str());
    RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex1.what());
    error_publisher_->addOneshotError("CreateClassException for the plugin " + _plugin_address_ + ": " + std::string(ex1.what()));
    error_publisher_->flushAndShutdown();
  }
  catch (pluginlib::PluginlibException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "PluginlibException for the plugin '%s'", _plugin_address_.c_str());
    RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex.what());
    error_publisher_->addOneshotError("PluginlibException for the plugin " + _plugin_address_ + ": " + std::string(ex.what()));
    error_publisher_->flushAndShutdown();
  }

  // | ------------------ initialize the plugin ----------------- |

  // the plugin's params are under mrs_uav_hw_api/plugin/
  rclcpp::Node::SharedPtr plugin_node = node_->create_sub_node("mrs_uav_hw_api")->create_sub_node("plugin");

  common_handlers_->plugin_param_loader = std::make_shared<mrs_lib::ParamLoader>(plugin_node, _plugin_address_);
  common_handlers_->plugin_param_loader->copyYamls(*param_loader_);

  common_handlers_->requested_capabilities = requested_capabilities_;

  hw_api_->initialize(plugin_node, common_handlers_);

  // | -------------------- plugin's topics --------------------- |

  capabilities_ = hw_api_->getCapabilities();

  topic_list_t not_subscribed;
  topic_list_t not_published;

  initCommandSubscribers(shopts, not_subscribed);
  initOutputPublishers(not_published);

  logDisabledTopics(not_subscribed, not_published);

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

//}

/* shutdown() //{ */

void HwApiManager::shutdown() {

  std::cout << "HwApiManager: shutdown(): called" << std::endl;

  if (!is_initialized_) {
    return;
  }

  timer_diagnostics_->stop();
  timer_mode_->stop();

  std::cout << "HwApiManager: calling destroy() on hw api plugin" << std::endl;

  hw_api_->destroy();

  std::cout << "HwApiManager: unloading hw api plugin" << std::endl;

  hw_api_.reset();

  std::cout << "HwApiManager: finished shutdown()" << std::endl;
}

//}

// | --------------------- topic callbacks -------------------- |

/* callbackActuatorCmd() //{ */

void HwApiManager::callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackActuatorCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'control group' command!");
  }
}

//}

/* callbackControlGroupCmd() //{ */

void HwApiManager::callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackControlGroupCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'control group' command!");
  }
}

//}

/* callbackAttitudeRateCmd() //{ */

void HwApiManager::callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAttitudeRateCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'attitude rate' command!");
  }
}

//}

/* callbackAttitudeCmd() //{ */

void HwApiManager::callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAttitudeCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'attitude' command!");
  }
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

void HwApiManager::callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAccelerationHdgRateCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'acceleration+hdg rate' command!");
  }
}

//}

/* callbackAccelerationHdgCmd() //{ */

void HwApiManager::callbackAccelerationHdgCmd(const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackAccelerationHdgCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'acceleration+hdg' command!");
  }
}

//}

/* callbackVelocityHdgRateCmd() //{ */

void HwApiManager::callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackVelocityHdgRateCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'velocity+hdg rate' command!");
  }
}

//}

/* callbackVelocityHdgCmd() //{ */

void HwApiManager::callbackVelocityHdgCmd(const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackVelocityHdgCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'velocity+hdg' command!");
  }
}

//}

/* callbackPositionCmd() //{ */

void HwApiManager::callbackPositionCmd(const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  bool result = hw_api_->callbackPositionCmd(msg);

  if (!result) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the currently loaded HW API does not implement the 'position' command!");
  }
}

//}

/* callbackTrackerCmd() //{ */

void HwApiManager::callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  hw_api_->callbackTrackerCmd(msg);
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackArming() //{ */

bool HwApiManager::callbackArming(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                  const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "%s", request->data ? "arming" : "disarming");

  auto [success, message] = hw_api_->callbackArming(request->data);

  response->success = success;
  response->message = message;

  return true;
}

//}

/* callbackOffboard() //{ */

bool HwApiManager::callbackOffboard([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    const std::shared_ptr<std_srvs::srv::Trigger::Response>                 response) {

  if (!is_initialized_) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "switching to offboard");

  auto [success, message] = hw_api_->callbackOffboard();

  response->success = success;
  response->message = message;

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerStatus() //{ */

void HwApiManager::timerStatus() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "timerStatus() spinning");

  mrs_msgs::msg::HwApiStatus status = hw_api_->getStatus();

  ph_status_.publish(status);

  if (!status.connected) {
    error_publisher_->addGeneralError(error_type_t::not_connected, "not connected");
  }

  if (status.connected) {

    std_msgs::msg::Empty msg;
    ph_connected_.publish(msg);
  }
}

//}

/* timerMode() //{ */

void HwApiManager::timerMode() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "timerCapabilities() spinning");

  mrs_msgs::msg::HwApiCapabilities diag = capabilities_;

  diag.stamp = clock_->now();

  ph_capabilities_.publish(diag);
}

//}

// | --------------------- common handlers -------------------- |

/* getUavName() //{ */

std::string HwApiManager::getUavName(void) {

  return _uav_name_;
}

//}

/* getBodyFrameName() //{ */

std::string HwApiManager::getBodyFrameName(void) {

  return _body_frame_name_;
}

//}

/* getBodyFrameName() //{ */

std::string HwApiManager::getWorldFrameName(void) {

  return _world_frame_name_;
}

//}

/* checkConfigKeys() //{ */

// the param loader ignores the keys nobody loads, so an obsolete or misspelled HW API parameter would be ignored silently
bool HwApiManager::checkConfigKeys(const std::vector<std::string> &config_files) {

  // old name -> new name, remove once no config uses the old layout
  const std::vector<std::pair<std::string, std::string>> obsolete_params = {
      {"plugin", "mrs_uav_hw_api/plugin"},
      {"plugin/input_mode", "mrs_uav_hw_api/input_mode"},
      {"plugin/outputs", "mrs_uav_hw_api/outputs"},
      {"input_mode", "mrs_uav_hw_api/input_mode"},
      {"outputs", "mrs_uav_hw_api/outputs"},
      {"hw_interface_plugin", "mrs_uav_hw_api/hw_interface_plugin"},
      {"publish_rate", "mrs_uav_hw_api/publish_rate"},
      {"timers", "mrs_uav_hw_api/timers"},
      {"body_frame_name", "mrs_uav_hw_api/body_frame_name"},
      {"world_frame_name", "mrs_uav_hw_api/world_frame_name"},
      {"mrs_uav_hw_api/plugin/input_mode", "mrs_uav_hw_api/input_mode"},
      {"mrs_uav_hw_api/plugin/outputs", "mrs_uav_hw_api/outputs"},
  };

  const std::set<std::string> hw_api_keys = {
      "version", "hw_interface_plugin", "body_frame_name", "world_frame_name", "timers", "publish_rate", "input_mode", "outputs", "plugin"};

  const std::set<std::string> input_mode_keys = {"actuators",        "control_group",     "attitude_rate", "attitude", "acceleration_hdg_rate",
                                                 "acceleration_hdg", "velocity_hdg_rate", "velocity_hdg",  "position"};

  const std::set<std::string> outputs_keys = {
      "distance_sensor", "gnss",        "gnss_status", "rtk",           "ground_truth", "imu",         "altitude", "magnetometer_heading",
      "magnetic_field",  "rc_channels", "rc_rssi",     "battery_state", "position",     "orientation", "velocity", "angular_velocity",
      "odometry"};

  bool success = true;

  const auto report = [&](const std::string &message) {
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    error_publisher_->addOneshotError(message);
    success = false;
  };

  for (const auto &config_file : config_files) {

    YAML::Node config;

    try {
      config = YAML::LoadFile(config_file);
    }
    catch (const YAML::Exception &e) {
      report("could not parse '" + config_file + "': " + e.what());
      continue;
    }

    if (!config.IsMap()) {
      continue;
    }

    for (const auto &[old_name, new_name] : obsolete_params) {
      if (hasYamlKey(config, old_name)) {
        report("obsolete parameter '" + old_name + "' found in '" + config_file + "', use '" + new_name + "' instead");
      }
    }

    // a missing key gives an invalid node: test it with operator bool before calling IsMap() on it
    const auto check_known = [&](const YAML::Node &yaml, const std::set<std::string> &known, const std::string &prefix) {
      if (!yaml || !yaml.IsMap()) {
        return;
      }

      for (const auto &item : yaml) {

        // a complex key (e.g. '[a, b]') does not convert to a string, print it as yaml
        const std::string key = item.first.IsScalar() ? item.first.Scalar() : YAML::Dump(item.first);

        if (!known.count(key)) {
          report("unknown parameter '" + prefix + key + "' found in '" + config_file + "'");
        }
      }
    };

    const YAML::Node hw_api = config["mrs_uav_hw_api"];

    check_known(hw_api, hw_api_keys, "mrs_uav_hw_api/");

    if (hw_api && hw_api.IsMap()) {
      check_known(hw_api["input_mode"], input_mode_keys, "mrs_uav_hw_api/input_mode/");
      check_known(hw_api["outputs"], outputs_keys, "mrs_uav_hw_api/outputs/");
    }
  }

  return success;
}

//}

/* hasYamlKey() //{ */

// whether the yaml has the key path "a/b/c"
bool HwApiManager::hasYamlKey(const YAML::Node &yaml, const std::string &path) {

  if (!yaml.IsMap()) {
    return false;
  }

  const size_t     separator = path.find('/');
  const YAML::Node child     = yaml[path.substr(0, separator)];

  if (!child) {
    return false;
  }

  return separator == std::string::npos || hasYamlKey(child, path.substr(separator + 1));
}

//}

/* loadRequestedCapabilities() //{ */

// compulsory, the plugin's config defines all of them
void HwApiManager::loadRequestedCapabilities(const std::string &yaml_prefix) {

  param_loader_->loadParam(yaml_prefix + "input_mode/actuators", requested_capabilities_.accepts_actuator_cmd);
  param_loader_->loadParam(yaml_prefix + "input_mode/control_group", requested_capabilities_.accepts_control_group_cmd);
  param_loader_->loadParam(yaml_prefix + "input_mode/attitude_rate", requested_capabilities_.accepts_attitude_rate_cmd);
  param_loader_->loadParam(yaml_prefix + "input_mode/attitude", requested_capabilities_.accepts_attitude_cmd);
  param_loader_->loadParam(yaml_prefix + "input_mode/acceleration_hdg_rate", requested_capabilities_.accepts_acceleration_hdg_rate_cmd);
  param_loader_->loadParam(yaml_prefix + "input_mode/acceleration_hdg", requested_capabilities_.accepts_acceleration_hdg_cmd);
  param_loader_->loadParam(yaml_prefix + "input_mode/velocity_hdg_rate", requested_capabilities_.accepts_velocity_hdg_rate_cmd);
  param_loader_->loadParam(yaml_prefix + "input_mode/velocity_hdg", requested_capabilities_.accepts_velocity_hdg_cmd);
  param_loader_->loadParam(yaml_prefix + "input_mode/position", requested_capabilities_.accepts_position_cmd);

  param_loader_->loadParam(yaml_prefix + "outputs/distance_sensor", requested_capabilities_.produces_distance_sensor);
  param_loader_->loadParam(yaml_prefix + "outputs/gnss", requested_capabilities_.produces_gnss);
  param_loader_->loadParam(yaml_prefix + "outputs/gnss_status", requested_capabilities_.produces_gnss_status);
  param_loader_->loadParam(yaml_prefix + "outputs/rtk", requested_capabilities_.produces_rtk);
  param_loader_->loadParam(yaml_prefix + "outputs/ground_truth", requested_capabilities_.produces_ground_truth);
  param_loader_->loadParam(yaml_prefix + "outputs/imu", requested_capabilities_.produces_imu);
  param_loader_->loadParam(yaml_prefix + "outputs/altitude", requested_capabilities_.produces_altitude);
  param_loader_->loadParam(yaml_prefix + "outputs/magnetometer_heading", requested_capabilities_.produces_magnetometer_heading);
  param_loader_->loadParam(yaml_prefix + "outputs/magnetic_field", requested_capabilities_.produces_magnetic_field);
  param_loader_->loadParam(yaml_prefix + "outputs/rc_channels", requested_capabilities_.produces_rc_channels);
  param_loader_->loadParam(yaml_prefix + "outputs/rc_rssi", requested_capabilities_.produces_rc_rssi);
  param_loader_->loadParam(yaml_prefix + "outputs/battery_state", requested_capabilities_.produces_battery_state);
  param_loader_->loadParam(yaml_prefix + "outputs/position", requested_capabilities_.produces_position);
  param_loader_->loadParam(yaml_prefix + "outputs/orientation", requested_capabilities_.produces_orientation);
  param_loader_->loadParam(yaml_prefix + "outputs/velocity", requested_capabilities_.produces_velocity);
  param_loader_->loadParam(yaml_prefix + "outputs/angular_velocity", requested_capabilities_.produces_angular_velocity);
  param_loader_->loadParam(yaml_prefix + "outputs/odometry", requested_capabilities_.produces_odometry);
}

//}

/* useCapability() //{ */

// whether to create the topic: requested and provided, otherwise collects it as disabled or warns
bool HwApiManager::useCapability(const bool requested, bool &capability, const std::string &key, const std::string &topic, topic_list_t &disabled) {

  if (!requested) {
    disabled.push_back({key, node_->get_node_topics_interface()->resolve_topic_name(topic)});
  } else if (!capability) {
    RCLCPP_WARN(node_->get_logger(), "%s is requested but not provided by the plugin", key.c_str());
  }

  capability = requested && capability;

  return capability;
}

//}

/* initCommandSubscribers() //{ */

void HwApiManager::initCommandSubscribers(const mrs_lib::SubscriberHandlerOptions &shopts, topic_list_t &not_subscribed) {

  if (useCapability(requested_capabilities_.accepts_actuator_cmd, capabilities_.accepts_actuator_cmd, "input_mode/actuators", "~/actuator_cmd",
                    not_subscribed)) {
    sh_actuator_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiActuatorCmd>(shopts, "~/actuator_cmd", &HwApiManager::callbackActuatorCmd, this);
  }

  if (useCapability(requested_capabilities_.accepts_control_group_cmd, capabilities_.accepts_control_group_cmd, "input_mode/control_group",
                    "~/control_group_cmd", not_subscribed)) {
    sh_control_group_cmd_ =
        mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiControlGroupCmd>(shopts, "~/control_group_cmd", &HwApiManager::callbackControlGroupCmd, this);
  }

  if (useCapability(requested_capabilities_.accepts_attitude_cmd, capabilities_.accepts_attitude_cmd, "input_mode/attitude", "~/attitude_cmd",
                    not_subscribed)) {
    sh_attitude_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAttitudeCmd>(shopts, "~/attitude_cmd", &HwApiManager::callbackAttitudeCmd, this);
  }

  if (useCapability(requested_capabilities_.accepts_attitude_rate_cmd, capabilities_.accepts_attitude_rate_cmd, "input_mode/attitude_rate",
                    "~/attitude_rate_cmd", not_subscribed)) {
    sh_attitude_rate_cmd_ =
        mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAttitudeRateCmd>(shopts, "~/attitude_rate_cmd", &HwApiManager::callbackAttitudeRateCmd, this);
  }

  if (useCapability(requested_capabilities_.accepts_acceleration_hdg_rate_cmd, capabilities_.accepts_acceleration_hdg_rate_cmd,
                    "input_mode/acceleration_hdg_rate", "~/acceleration_hdg_rate_cmd", not_subscribed)) {
    sh_acceleration_hdg_rate_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAccelerationHdgRateCmd>(shopts, "~/acceleration_hdg_rate_cmd",
                                                                                                           &HwApiManager::callbackAccelerationHdgRateCmd, this);
  }

  if (useCapability(requested_capabilities_.accepts_acceleration_hdg_cmd, capabilities_.accepts_acceleration_hdg_cmd, "input_mode/acceleration_hdg",
                    "~/acceleration_hdg_cmd", not_subscribed)) {
    sh_acceleration_hdg_cmd_ =
        mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAccelerationHdgCmd>(shopts, "~/acceleration_hdg_cmd", &HwApiManager::callbackAccelerationHdgCmd, this);
  }

  if (useCapability(requested_capabilities_.accepts_velocity_hdg_rate_cmd, capabilities_.accepts_velocity_hdg_rate_cmd, "input_mode/velocity_hdg_rate",
                    "~/velocity_hdg_rate_cmd", not_subscribed)) {
    sh_velocity_hdg_rate_cmd_ =
        mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiVelocityHdgRateCmd>(shopts, "~/velocity_hdg_rate_cmd", &HwApiManager::callbackVelocityHdgRateCmd, this);
  }

  if (useCapability(requested_capabilities_.accepts_velocity_hdg_cmd, capabilities_.accepts_velocity_hdg_cmd, "input_mode/velocity_hdg", "~/velocity_hdg_cmd",
                    not_subscribed)) {
    sh_velocity_hdg_cmd_ =
        mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiVelocityHdgCmd>(shopts, "~/velocity_hdg_cmd", &HwApiManager::callbackVelocityHdgCmd, this);
  }

  if (useCapability(requested_capabilities_.accepts_position_cmd, capabilities_.accepts_position_cmd, "input_mode/position", "~/position_cmd",
                    not_subscribed)) {
    sh_position_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiPositionCmd>(shopts, "~/position_cmd", &HwApiManager::callbackPositionCmd, this);
  }
}

//}

/* initOutputPublishers() //{ */

void HwApiManager::initOutputPublishers(topic_list_t &not_published) {

  if (useCapability(requested_capabilities_.produces_gnss, capabilities_.produces_gnss, "outputs/gnss", "~/gnss", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_gnss_rate_;

    ph_gnss_ = mrs_lib::PublisherHandler<sensor_msgs::msg::NavSatFix>(opts, "~/gnss");
  }

  if (useCapability(requested_capabilities_.produces_gnss_status, capabilities_.produces_gnss_status, "outputs/gnss_status", "~/gnss_status", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_gnss_status_rate_;

    ph_gnss_status_ = mrs_lib::PublisherHandler<mrs_msgs::msg::GpsInfo>(opts, "~/gnss_status");
  }

  if (useCapability(requested_capabilities_.produces_rtk, capabilities_.produces_rtk, "outputs/rtk", "~/rtk", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_rtk_rate_;

    ph_rtk_ = mrs_lib::PublisherHandler<mrs_msgs::msg::RtkGps>(opts, "~/rtk");
  }

  if (useCapability(requested_capabilities_.produces_distance_sensor, capabilities_.produces_distance_sensor, "outputs/distance_sensor", "~/distance_sensor",
                    not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_distance_sensor_rate_;

    ph_distance_sensor_ = mrs_lib::PublisherHandler<sensor_msgs::msg::Range>(opts, "~/distance_sensor");
  }

  if (useCapability(requested_capabilities_.produces_magnetometer_heading, capabilities_.produces_magnetometer_heading, "outputs/magnetometer_heading",
                    "~/mag_heading", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_mag_heading_rate_;

    ph_mag_heading_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(opts, "~/mag_heading");
  }

  if (useCapability(requested_capabilities_.produces_magnetic_field, capabilities_.produces_magnetic_field, "outputs/magnetic_field", "~/magnetic_field",
                    not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_mag_magnetic_field_rate_;

    ph_mag_magnetic_field_ = mrs_lib::PublisherHandler<sensor_msgs::msg::MagneticField>(opts, "~/magnetic_field");
  }

  if (useCapability(requested_capabilities_.produces_altitude, capabilities_.produces_altitude, "outputs/altitude", "~/altitude", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_altitude_rate_;

    ph_altitude_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiAltitude>(opts, "~/altitude");
  }

  if (useCapability(requested_capabilities_.produces_imu, capabilities_.produces_imu, "outputs/imu", "~/imu", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_imu_rate_;

    ph_imu_ = mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>(opts, "~/imu");
  }

  if (useCapability(requested_capabilities_.produces_rc_channels, capabilities_.produces_rc_channels, "outputs/rc_channels", "~/rc_channels", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_rc_channels_rate_;

    ph_rc_channels_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiRcChannels>(opts, "~/rc_channels");
  }

  if (useCapability(requested_capabilities_.produces_rc_rssi, capabilities_.produces_rc_rssi, "outputs/rc_rssi", "~/rc_rssi", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_rc_channels_rate_;

    ph_rc_rssi_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiRcRssi>(opts, "~/rc_rssi");
  }

  if (useCapability(requested_capabilities_.produces_battery_state, capabilities_.produces_battery_state, "outputs/battery_state", "~/battery_state",
                    not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_battery_state_rate_;

    ph_battery_state_ = mrs_lib::PublisherHandler<sensor_msgs::msg::BatteryState>(opts, "~/battery_state");
  }

  if (useCapability(requested_capabilities_.produces_position, capabilities_.produces_position, "outputs/position", "~/position", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_position_rate_;

    ph_position_ = mrs_lib::PublisherHandler<geometry_msgs::msg::PointStamped>(opts, "~/position");
  }

  if (useCapability(requested_capabilities_.produces_orientation, capabilities_.produces_orientation, "outputs/orientation", "~/orientation", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_orientation_rate_;

    ph_orientation_ = mrs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped>(opts, "~/orientation");
  }

  if (useCapability(requested_capabilities_.produces_velocity, capabilities_.produces_velocity, "outputs/velocity", "~/velocity", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_velocity_rate_;

    ph_velocity_ = mrs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>(opts, "~/velocity");
  }

  if (useCapability(requested_capabilities_.produces_angular_velocity, capabilities_.produces_angular_velocity, "outputs/angular_velocity",
                    "~/angular_velocity", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_angular_velocity_rate_;

    ph_angular_velocity_ = mrs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>(opts, "~/angular_velocity");
  }

  if (useCapability(requested_capabilities_.produces_odometry, capabilities_.produces_odometry, "outputs/odometry", "~/odometry", not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_odometry_rate_;

    ph_odometry_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(opts, "~/odometry");
  }

  if (useCapability(requested_capabilities_.produces_ground_truth, capabilities_.produces_ground_truth, "outputs/ground_truth", "~/ground_truth",
                    not_published)) {
    mrs_lib::PublisherHandlerOptions opts;

    opts.node          = node_;
    opts.throttle_rate = _pub_ground_truth_rate_;

    ph_ground_truth_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(opts, "~/ground_truth");
  }
}

//}

/* logDisabledTopics() //{ */

void HwApiManager::logDisabledTopics(const topic_list_t &not_subscribed, const topic_list_t &not_published) {

  // print the topics in one column
  int width = 0;

  for (const topic_list_t *list : {&not_subscribed, &not_published}) {
    for (const auto &[key, topic] : *list) {
      width = std::max(width, int(key.size()) + 1);
    }
  }

  auto logList = [&](const std::string &header, const topic_list_t &list) {
    if (list.empty()) {
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "%s", header.c_str());

    for (const auto &[key, topic] : list) {
      RCLCPP_INFO(node_->get_logger(), "  %-*s %s", width, (key + ":").c_str(), topic.c_str());
    }
  };

  logList("disabled in the config, not subscribing to:", not_subscribed);
  logList("disabled in the config, not publishing:", not_published);
}

//}

// | ----------------------- publishers ----------------------- |

/* publishOutput() //{ */

template <typename MessageType>
void HwApiManager::publishOutput(const bool produced, const std::string &key, mrs_lib::PublisherHandler<MessageType> &publisher, const MessageType &msg) {

  // only what is requested and provided
  if (!produced) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the plugin publishes '%s', which is not requested or not provided, dropping it", key.c_str());
    return;
  }

  publisher.publish(msg);
}

//}

/* publishGNSS() //{ */

void HwApiManager::publishGNSS(const sensor_msgs::msg::NavSatFix &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_gnss, "outputs/gnss", ph_gnss_, msg);
}

//}

/* publishGNSSStatus() //{ */

void HwApiManager::publishGNSSStatus(const mrs_msgs::msg::GpsInfo &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_gnss_status, "outputs/gnss_status", ph_gnss_status_, msg);
}

//}

/* publishRTK() //{ */

void HwApiManager::publishRTK(const mrs_msgs::msg::RtkGps &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_rtk, "outputs/rtk", ph_rtk_, msg);
}

//}

/* publishIMU() //{ */

void HwApiManager::publishIMU(const sensor_msgs::msg::Imu &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_imu, "outputs/imu", ph_imu_, msg);
}

//}

/* publishDistanceSensor() //{ */

void HwApiManager::publishDistanceSensor(const sensor_msgs::msg::Range &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_distance_sensor, "outputs/distance_sensor", ph_distance_sensor_, msg);
}

//}

/* publishAltitude() //{ */

void HwApiManager::publishAltitude(const mrs_msgs::msg::HwApiAltitude &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_altitude, "outputs/altitude", ph_altitude_, msg);
}

//}

/* publishMagnetometerHeading() //{ */

void HwApiManager::publishMagnetometerHeading(const mrs_msgs::msg::Float64Stamped &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_magnetometer_heading, "outputs/magnetometer_heading", ph_mag_heading_, msg);
}

//}

/* publishMagneticField() //{ */

void HwApiManager::publishMagneticField(const sensor_msgs::msg::MagneticField &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_magnetic_field, "outputs/magnetic_field", ph_mag_magnetic_field_, msg);
}

//}

/* publishStatus() //{ */

void HwApiManager::publishStatus(const mrs_msgs::msg::HwApiStatus &msg) {

  if (!is_initialized_) {
    return;
  }

  ph_status_.publish(msg);
}

//}

/* publishRcChannels() //{ */

void HwApiManager::publishRcChannels(const mrs_msgs::msg::HwApiRcChannels &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_rc_channels, "outputs/rc_channels", ph_rc_channels_, msg);
}

//}

/* publishRcRssi() //{ */

void HwApiManager::publishRcRssi(const mrs_msgs::msg::HwApiRcRssi &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_rc_rssi, "outputs/rc_rssi", ph_rc_rssi_, msg);
}

//}

/* publishPosition() //{ */

void HwApiManager::publishPosition(const geometry_msgs::msg::PointStamped &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_position, "outputs/position", ph_position_, msg);
}

//}

/* publishOrientation() //{ */

void HwApiManager::publishOrientation(const geometry_msgs::msg::QuaternionStamped &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_orientation, "outputs/orientation", ph_orientation_, msg);
}

//}

/* publishVelocity() //{ */

void HwApiManager::publishVelocity(const geometry_msgs::msg::Vector3Stamped &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_velocity, "outputs/velocity", ph_velocity_, msg);
}

//}

/* publishAngularVelocity() //{ */

void HwApiManager::publishAngularVelocity(const geometry_msgs::msg::Vector3Stamped &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_angular_velocity, "outputs/angular_velocity", ph_angular_velocity_, msg);
}

//}

/* publishOdometry() //{ */

void HwApiManager::publishOdometry(const nav_msgs::msg::Odometry &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_odometry, "outputs/odometry", ph_odometry_, msg);
}

//}

/* publishGroundTruth() //{ */

void HwApiManager::publishGroundTruth(const nav_msgs::msg::Odometry &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_ground_truth, "outputs/ground_truth", ph_ground_truth_, msg);
}

//}

/* publishBatteryState() //{ */

void HwApiManager::publishBatteryState(const sensor_msgs::msg::BatteryState &msg) {

  if (!is_initialized_) {
    return;
  }

  publishOutput(capabilities_.produces_battery_state, "outputs/battery_state", ph_battery_state_, msg);
}

//}

} // namespace mrs_uav_hw_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_hw_api::HwApiManager)
