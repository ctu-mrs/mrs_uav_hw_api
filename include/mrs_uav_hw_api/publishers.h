#ifndef PUBLISHERS_H
#define PUBLISHERS_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>

#include <mrs_msgs/Float64Stamped.h>

#include <mrs_msgs/HwApiDiagnostics.h>

#include <nav_msgs/Odometry.h>

namespace mrs_uav_hw_api
{

//}

typedef std::function<void(const sensor_msgs::NavSatFix &msg)>     publishGNSS_t;
typedef std::function<void(const sensor_msgs::Imu &msg)>           publishIMU_t;
typedef std::function<void(const sensor_msgs::NavSatStatus &msg)>  publishGNSSStatus_t;
typedef std::function<void(const sensor_msgs::Range &msg)>         publishRange_t;
typedef std::function<void(const mrs_msgs::Float64Stamped &msg)>   publishAltitude_t;
typedef std::function<void(const mrs_msgs::Float64Stamped &msg)>   publishMagnetometerHeading_t;
typedef std::function<void(const nav_msgs::Odometry &msg)>         publishOdometryLocal_t;
typedef std::function<void(const mrs_msgs::HwApiDiagnostics &msg)> publishDiagnostics_t;

struct Publishers_t
{
  publishIMU_t                 publishIMU;
  publishGNSS_t                publishGNSS;
  publishGNSSStatus_t          publishGNSSStatus;
  publishRange_t               publishRange;
  publishAltitude_t            publishAltitude;
  publishMagnetometerHeading_t publishMagnetometerHeading;
  publishOdometryLocal_t       publishOdometryLocal;
  publishDiagnostics_t         publishDiagnostics;
};

}  // namespace mrs_uav_hw_api

#endif  // PUBLISHERS_H