#ifndef MRS_UAV_HW_API_H
#define MRS_UAV_HW_API_H

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_hw_api/common_handlers.h>

//}

namespace mrs_uav_hw_interface
{

class UAVHwInterface {

public:

  virtual ~UAVHwInterface() = 0;

  virtual void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) = 0;

  // | --------------------- topic callbacks -------------------- |

  virtual bool callbackActuatorCmd() = 0;

  virtual bool callbackAttitudeRateCmd() = 0;

  virtual bool callbackAttitudeCmd() = 0;

  virtual bool callbackAccelerationCmd() = 0;

  virtual bool callbackVelocityCmd() = 0;

  // | -------------------- service callbacks ------------------- |

  virtual bool callbackArmCmd() = 0;

  virtual bool callbackOffboardCmd() = 0;

};

}  // namespace mrs_uav_managers

#endif
