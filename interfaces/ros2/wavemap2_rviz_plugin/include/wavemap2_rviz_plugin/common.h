#ifndef WAVEMAP2_RVIZ_PLUGIN_COMMON_H_
#define WAVEMAP2_RVIZ_PLUGIN_COMMON_H_

#include <mutex>

#include <wavemap/core/map/map_base.h>

namespace wavemap::rviz_plugin {
struct MapAndMutex {
  MapBase::Ptr map;
  std::mutex mutex;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP2_RVIZ_PLUGIN_COMMON_H_
