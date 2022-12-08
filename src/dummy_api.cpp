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

namespace mrs_uav_pixhawk_api
{

/* class DummyApi //{ */

class DummyApi : public mrs_uav_hw_api::MrsUavHwApi {

public:
  ~DummyApi(){};

  void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers, const std::string &topic_prefix,
                  const std::string &uav_name);

  mrs_msgs::HwApiDiagnostics getDiagnostics();
  mrs_msgs::HwApiMode        getMode();

  // | --------------------- topic callbacks -------------------- |

  bool callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd &msg);
  bool callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd &msg);
  bool callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd &msg);
  bool callbackTranslationCmd(const mrs_msgs::HwApiTranslationCmd &msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);

  std::tuple<bool, std::string> callbackOffboard(const bool &request);

private:
  bool is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void DummyApi::initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers, const std::string &topic_prefix,
                          const std::string &uav_name) {

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

// | ----------------- main interface methods ----------------- |

mrs_msgs::HwApiDiagnostics DummyApi::getDiagnostics() {

  mrs_msgs::HwApiDiagnostics diag;

  diag.armed = false;
  diag.offboard = false;
  diag.running = false;

  return diag;
}

mrs_msgs::HwApiMode DummyApi::getMode() {

  mrs_msgs::HwApiDiagnostics diag;
}

}  // namespace mrs_uav_pixhawk_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_pixhawk_api::DummyApi, mrs_uav_hw_api::MrsUavHwApi)
