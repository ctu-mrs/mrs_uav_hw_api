#ifndef MRS_UAV_HW_API_H
#define MRS_UAV_HW_API_H

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_hw_api/common_handlers.h>

#include <mrs_msgs/HwApiControlGroupCmd.h>
#include <mrs_msgs/HwApiAttitudeCmd.h>
#include <mrs_msgs/HwApiAttitudeRateCmd.h>
#include <mrs_msgs/HwApiTranslationCmd.h>

//}

namespace mrs_uav_hw_api
{

class MrsUavHwApi {

public:
  virtual ~MrsUavHwApi() = 0;

  virtual void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) = 0;

  // | --------------------- topic callbacks -------------------- |

  virtual bool callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd &msg) = 0;

  virtual bool callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd &msg) = 0;

  virtual bool callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd &msg) = 0;

  virtual bool callbackTranslationCmd(const mrs_msgs::HwApiTranslationCmd &msg) = 0;

  // | -------------------- service callbacks ------------------- |

  virtual std::tuple<bool, std::string> callbackArming(const bool &request) = 0;

  virtual std::tuple<bool, std::string> callbackOffboard(const bool &request) = 0;
};

}  // namespace mrs_uav_hw_api

#endif
