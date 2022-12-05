#ifndef COMMON_HANDLERS_H
#define COMMON_HANDLERS_H

#include <mrs_uav_hw_api/publishers.h>
#include <mrs_lib/transformer.h>

namespace mrs_uav_hw_api
{

//}

struct CommonHandlers_t
{
  Publishers_t                          publishers;
  std::shared_ptr<mrs_lib::Transformer> transformer;
};


}  // namespace mrs_uav_hw_api

#endif  // COMMON_HANDLERS_H
