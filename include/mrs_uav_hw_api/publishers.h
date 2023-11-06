#ifndef PUBLISHERS_H
#define PUBLISHERS_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>

#include <mrs_msgs/Float64Stamped.h>

#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/HwApiRcChannels.h>
#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/RtkFixType.h>

#include <mrs_msgs/HwApiAltitude.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

namespace mrs_uav_hw_api
{

//}

typedef std::function<void(const sensor_msgs::NavSatFix &msg)>     publishGNSS_t;
typedef std::function<void(const sensor_msgs::NavSatStatus &msg)>  publishGNSSStatus_t;
typedef std::function<void(const mrs_msgs::RtkGps &msg)>           publishRTK_t;
typedef std::function<void(const mrs_msgs::HwApiAltitude &msg)>    publishAltitude_t;
typedef std::function<void(const mrs_msgs::Float64Stamped &msg)>   publishMagnetometerHeading_t;
typedef std::function<void(const sensor_msgs::MagneticField &msg)> publishMagneticField_t;
typedef std::function<void(const mrs_msgs::HwApiStatus &msg)>      publishStatus_t;
typedef std::function<void(const mrs_msgs::HwApiRcChannels &msg)>  publishRcChannels_t;
typedef std::function<void(const sensor_msgs::BatteryState &msg)>  publishBatteryState_t;
typedef std::function<void(const sensor_msgs::Imu &msg)>           publishIMU_t;

/**
 * @brief The distance sensor is expected to be pointing down towards the ground
 */
typedef std::function<void(const sensor_msgs::Range &msg)> publishDistanceSensor_t;

/**
 * @brief The UAV Velocity expressed in the UAV Body frame.
 */
typedef std::function<void(const geometry_msgs::Vector3Stamped &msg)>    publishVelocity_t;

typedef std::function<void(const geometry_msgs::PointStamped &msg)>      publishPosition_t;
typedef std::function<void(const geometry_msgs::QuaternionStamped &msg)> publishOrientation_t;
typedef std::function<void(const geometry_msgs::Vector3Stamped &msg)>    publishAngularVelocity_t;
typedef std::function<void(const nav_msgs::Odometry &msg)>               publishOdometry_t;

/**
 * @brief A structure with publisher methods provided to the HW API Plugin.
 * The plugin can call these methods at will to provided data to the MRS UAV System.
 * In case the HW API Plugin intents to publish data, the respective varible "produces_***"
 * in the HW API Capabilities of the plugin is supposed to be fill in.
 */
struct Publishers_t
{
  publishIMU_t                 publishIMU;
  publishGNSS_t                publishGNSS;
  publishGNSSStatus_t          publishGNSSStatus;
  publishRTK_t                 publishRTK;
  publishDistanceSensor_t      publishDistanceSensor;
  publishAltitude_t            publishAltitude;
  publishMagnetometerHeading_t publishMagnetometerHeading;
  publishMagneticField_t       publishMagneticField;
  publishStatus_t              publishStatus;
  publishRcChannels_t          publishRcChannels;
  publishBatteryState_t        publishBatteryState;
  publishPosition_t            publishPosition;
  publishOrientation_t         publishOrientation;
  publishVelocity_t            publishVelocity;
  publishAngularVelocity_t     publishAngularVelocity;
  publishOdometry_t            publishOdometry;
  publishOdometry_t            publishGroundTruth;
};

}  // namespace mrs_uav_hw_api

#endif  // PUBLISHERS_H
