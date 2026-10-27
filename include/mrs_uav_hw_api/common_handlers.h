#pragma once

#include <functional>

#include <mrs_uav_hw_api/publishers.h>
#include <mrs_lib/param_loader.h>
#include <mrs_msgs/msg/hw_api_capabilities.hpp>

namespace mrs_uav_hw_api
{

//}

typedef std::function<std::string(void)> getUavName_t;
typedef std::function<std::string(void)> getBodyFrameName_t;
typedef std::function<std::string(void)> getWorldFrameName_t;

/**
 * @brief A structure with methods and variables provided to the HW API Plugin.
 */
struct CommonHandlers_t
{
  /**
   * @brief A structure with publishers that the plugin should use to provide data to the MRS UAV System.
   */
  Publishers_t publishers;

  std::shared_ptr<mrs_lib::ParamLoader> main_param_loader;

  /**
   * @brief Param loader for the plugin's params, which are under mrs_uav_hw_api/plugin/ in the config files.
   */
  std::shared_ptr<mrs_lib::ParamLoader> plugin_param_loader;

  /**
   * @brief The inputs and outputs the configs request, filled before initialize(). The plugin provides a subset and returns it from getCapabilities().
   */
  mrs_msgs::msg::HwApiCapabilities requested_capabilities;

  /**
   * @brief Method for obtaining the current "UAV_NAME" (the one set from the $UAV_NAME env variable).
   */
  getUavName_t getUavName;

  /**
   * @brief Method for obtaining the body-fixed frame_id (as it is called by the MRS UAV System). This frame_id should be used when publishing data in the
   * body-fixed frame, e.g., the IMU data.
   */
  getBodyFrameName_t getBodyFrameName;

  /**
   * @brief Method for obtaining the world-frame for marking the flight-controller's data, e.g., the position of the UAV in the flight-controllers frame.
   */
  getWorldFrameName_t getWorldFrameName;
};

} // namespace mrs_uav_hw_api
