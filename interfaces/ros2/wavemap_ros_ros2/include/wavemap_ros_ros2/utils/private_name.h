#ifndef WAVEMAP_ROS_ROS2_UTILS_PRIVATE_NAME_H_
#define WAVEMAP_ROS_ROS2_UTILS_PRIVATE_NAME_H_

#include <string>

namespace wavemap {
// Resolve a topic or service name the way ROS1's private node handle did.
//
// ROS1 advertised these through nh_private, whose namespace is the node name,
// so `advertise("map")` became /wavemap/map. ROS2 resolves a relative name
// against the node's *namespace* instead, which would yield /map, so the
// tilde has to be explicit. See the "Topic/service namespacing" entry in
// claude/issues.md.
//
// Names the user wrote as absolute (leading '/') are passed through
// untouched, matching ROS1: nh_private.advertise("/foo") also resolved
// globally. A name that is already private ("~/...") is left alone too.
inline std::string privateName(const std::string& name) {
  if (name.empty() || name.front() == '/' || name.front() == '~') {
    return name;
  }
  return "~/" + name;
}
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_UTILS_PRIVATE_NAME_H_
