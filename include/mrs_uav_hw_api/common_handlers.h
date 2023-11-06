#ifndef COMMON_HANDLERS_H
#define COMMON_HANDLERS_H

#include <mrs_uav_hw_api/publishers.h>
#include <mrs_lib/transformer.h>

namespace mrs_uav_hw_api
{

//}

typedef std::function<std::string(void)> getUavName_t;
typedef std::function<std::string(void)> getBodyFrameName_t;
typedef std::function<std::string(void)> getWorldFrameName_t;

/**
 * @brief A tructure with methods and variables provided to the HW API Plugin.
 */
struct CommonHandlers_t
{
  /**
   * @brief A 
   */
  Publishers_t                          publishers;

  std::shared_ptr<mrs_lib::Transformer> transformer;

  getUavName_t        getUavName;
  getBodyFrameName_t  getBodyFrameName;
  getWorldFrameName_t getWorldFrameName;
};

}  // namespace mrs_uav_hw_api

#endif  // COMMON_HANDLERS_H
