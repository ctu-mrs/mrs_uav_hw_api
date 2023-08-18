#ifndef MRS_UAV_HW_API_H
#define MRS_UAV_HW_API_H

/* includes //{ */

#include <ros/ros.h>

#include <mrs_lib/subscribe_handler.h>

#include <mrs_uav_hw_api/common_handlers.h>

#include <mrs_msgs/HwApiActuatorCmd.h>
#include <mrs_msgs/HwApiControlGroupCmd.h>
#include <mrs_msgs/HwApiAttitudeCmd.h>
#include <mrs_msgs/HwApiAttitudeRateCmd.h>
#include <mrs_msgs/HwApiAccelerationHdgRateCmd.h>
#include <mrs_msgs/HwApiAccelerationHdgCmd.h>
#include <mrs_msgs/HwApiVelocityHdgRateCmd.h>
#include <mrs_msgs/HwApiVelocityHdgCmd.h>
#include <mrs_msgs/HwApiPositionCmd.h>
#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/HwApiCapabilities.h>
#include <mrs_msgs/TrackerCommand.h>

//}

namespace mrs_uav_hw_api
{

class MrsUavHwApi {

public:
  virtual ~MrsUavHwApi() = 0;

  virtual void initialize(const ros::NodeHandle& parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) = 0;

  virtual mrs_msgs::HwApiStatus       getStatus()       = 0;
  virtual mrs_msgs::HwApiCapabilities getCapabilities() = 0;

  // | --------------------- topic callbacks -------------------- |

  virtual bool callbackActuatorCmd(const mrs_msgs::HwApiActuatorCmd::ConstPtr msg) = 0;
  virtual bool callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg) = 0;
  virtual bool callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg) = 0;
  virtual bool callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg) = 0;
  virtual bool callbackAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg) = 0;
  virtual bool callbackAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg) = 0;
  virtual bool callbackVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg) = 0;
  virtual bool callbackVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg) = 0;
  virtual bool callbackPositionCmd(const mrs_msgs::HwApiPositionCmd::ConstPtr msg) = 0;
  virtual void callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg) = 0;

  // | -------------------- service callbacks ------------------- |

  virtual std::tuple<bool, std::string> callbackArming(const bool& request) = 0;

  virtual std::tuple<bool, std::string> callbackOffboard(void) = 0;
};

// A pure virtual destructor requires a function body.
MrsUavHwApi::~MrsUavHwApi(){};

}  // namespace mrs_uav_hw_api

#endif
