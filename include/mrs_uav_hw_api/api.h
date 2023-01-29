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
#include <mrs_msgs/HwApiPositionCmd.h>
#include <mrs_msgs/HwApiVelocityCmd.h>
#include <mrs_msgs/HwApiAccelerationCmd.h>
#include <mrs_msgs/HwApiDiagnostics.h>
#include <mrs_msgs/HwApiMode.h>

//}

namespace mrs_uav_hw_api
{

class MrsUavHwApi {

public:
  virtual ~MrsUavHwApi() = 0;

  virtual void initialize(const ros::NodeHandle& parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers, const std::string& topic_prefix,
                          const std::string& uav_name) = 0;

  virtual mrs_msgs::HwApiDiagnostics getDiagnostics() = 0;
  virtual mrs_msgs::HwApiMode        getMode()        = 0;

  // | --------------------- topic callbacks -------------------- |

  virtual bool callbackActuatorCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>& wrp) = 0;

  virtual bool callbackControlGroupCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>& wrp) = 0;

  virtual bool callbackAttitudeRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp) = 0;

  virtual bool callbackAttitudeCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp) = 0;

  virtual bool callbackAccelerationCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationCmd>& wrp) = 0;

  virtual bool callbackVelocityCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityCmd>& wrp) = 0;

  virtual bool callbackPositionCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>& wrp) = 0;

  // | -------------------- service callbacks ------------------- |

  virtual std::tuple<bool, std::string> callbackArming(const bool& request) = 0;

  virtual std::tuple<bool, std::string> callbackOffboard(void) = 0;
};

// A pure virtual destructor requires a function body.
MrsUavHwApi::~MrsUavHwApi(){};

}  // namespace mrs_uav_hw_api

#endif
