#ifndef PUBLISHERS_H
#define PUBLISHERS_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>

#include <mrs_msgs/Float64Stamped.h>

#include <nav_msgs/Odometry.h>

namespace mrs_uav_hw_api
{

//}

struct Publishers_t
{
  typedef std::function<bool(const sensor_msgs::Imu &msg)>          publishImu_t;
  typedef std::function<bool(const sensor_msgs::NavSatFix &msg)>    publishGps_t;
  typedef std::function<bool(const sensor_msgs::NavSatStatus &msg)> publishGpsStatus_t;
  typedef std::function<bool(const sensor_msgs::Range &msg)>        publishHeightSensor_t;
  typedef std::function<bool(const mrs_msgs::Float64Stamped &msg)>  publishAltitude_t;
  typedef std::function<bool(const mrs_msgs::Float64Stamped &msg)>  publishMagnetometerHeading_t;
  typedef std::function<bool(const nav_msgs::Odometry &msg)>        publishOdometryLocal_t;
};


}  // namespace mrs_uav_hw_api

#endif  // PUBLISHERS_H
