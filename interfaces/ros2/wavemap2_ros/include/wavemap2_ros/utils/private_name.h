#ifndef WAVEMAP2_ROS_UTILS_PRIVATE_NAME_H_
#define WAVEMAP2_ROS_UTILS_PRIVATE_NAME_H_

#include <string>

namespace wavemap {
// Resolve a topic or service name in the node's private namespace
// NOTE: ROS2 resolves relative names against the node's namespace, so "map"
//       would become /map. Prefixing it with "~/" makes it /wavemap/map,
//       matching the ROS1 interface. Absolute names (leading '/') and names
//       that are already private ("~/...") are passed through untouched.
inline std::string privateName(const std::string& name) {
  if (name.empty() || name.front() == '/' || name.front() == '~') {
    return name;
  }
  return "~/" + name;
}
}  // namespace wavemap

#endif  // WAVEMAP2_ROS_UTILS_PRIVATE_NAME_H_
