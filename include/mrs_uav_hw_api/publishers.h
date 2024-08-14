#ifndef PUBLISHERS_H
#define PUBLISHERS_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>

#include <mrs_msgs/Float64Stamped.h>

#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/HwApiRcChannels.h>
#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/RtkFixType.h>
#include <mrs_msgs/GpsInfo.h>

#include <mrs_msgs/HwApiAltitude.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

namespace mrs_uav_hw_api
{

//}

typedef std::function<void(const sensor_msgs::NavSatFix &msg)>           publishGNSS_t;
typedef std::function<void(const mrs_msgs::GpsInfo &msg)>        publishGNSSStatus_t;
typedef std::function<void(const mrs_msgs::RtkGps &msg)>                 publishRTK_t;
typedef std::function<void(const mrs_msgs::HwApiAltitude &msg)>          publishAltitude_t;
typedef std::function<void(const mrs_msgs::Float64Stamped &msg)>         publishMagnetometerHeading_t;
typedef std::function<void(const sensor_msgs::MagneticField &msg)>       publishMagneticField_t;
typedef std::function<void(const mrs_msgs::HwApiStatus &msg)>            publishStatus_t;
typedef std::function<void(const mrs_msgs::HwApiRcChannels &msg)>        publishRcChannels_t;
typedef std::function<void(const sensor_msgs::BatteryState &msg)>        publishBatteryState_t;
typedef std::function<void(const sensor_msgs::Imu &msg)>                 publishIMU_t;
typedef std::function<void(const sensor_msgs::Range &msg)>               publishDistanceSensor_t;
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
  /**
   * @brief Publisher for the raw IMU data.
   */
  publishIMU_t publishIMU;

  /**
   * @brief Publisher for the main GNSS data.
   */
  publishGNSS_t publishGNSS;

  /**
   * @brief Publisher for the GNSS status message.
   */
  publishGNSSStatus_t publishGNSSStatus;

  /**
   * @brief Publisher for the RTK GNSS data.
   */
  publishRTK_t publishRTK;

  /**
   * @brief Publisher for the distance sensor readings. The distance sensor is expected to be pointing down towards the ground.
   */
  publishDistanceSensor_t publishDistanceSensor;

  /**
   * @brief Altitude of the UAV in AMSL (Above Mean Seal Level).
   */
  publishAltitude_t publishAltitude;

  /**
   * @brief Publisher for the estimated magnetometer heading.
   */
  publishMagnetometerHeading_t publishMagnetometerHeading;

  /**
   * @brief Publisher for the RAW magnetometer readings.
   */
  publishMagneticField_t publishMagneticField;

  /**
   * @brief Publisher for the HW API Plugin status.
   */
  publishStatus_t publishStatus;

  /**
   * @brief Publisher for the RC Channels received by the flight controller.
   */
  publishRcChannels_t publishRcChannels;

  /**
   * @brief Publisher for the UAV battery state.
   */
  publishBatteryState_t publishBatteryState;

  /**
   * @brief Publisher for the UAV position as estimated by the flight controller. The position is supposed to be expressed in the "world frame" provided by the
   * common handlers.
   */
  publishPosition_t publishPosition;

  /**
   * @brief Publisher for the UAV orientation. The orientation is supposed to be expressed in the "world frame" provided by the common handlers.
   */
  publishOrientation_t publishOrientation;

  /**
   * @brief Publisher for the UAV velocity expressed in the body-fixed frame, provided by the common handlers.
   */
  publishVelocity_t publishVelocity;

  /**
   * @brief Publisher for the intrinsic angular velocity. This can be already pre-filtered. The angular velocity is supposed to be expressed in the "body-fixed"
   * frame provided in the common handlers.
   */
  publishAngularVelocity_t publishAngularVelocity;

  /**
   * @brief Publisher for a general "odometry" message that can be used for vizualizing the state of the UAV, as produced by the flight controller. The MRS UAV
   * System will use this when running the "passthrough_ state estimator.
   */
  publishOdometry_t publishOdometry;

  /**
   * @brief Publisher for ground truth odometry which can be provided by, e.g., a simulator or a MoCap system. The MRS UAV System will use this when running the
   * "ground_truth" state estimator.
   */
  publishOdometry_t publishGroundTruth;
};

}  // namespace mrs_uav_hw_api

#endif  // PUBLISHERS_H
