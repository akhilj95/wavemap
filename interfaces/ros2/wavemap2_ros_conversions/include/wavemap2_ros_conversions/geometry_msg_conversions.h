#ifndef WAVEMAP_ROS_CONVERSIONS_ROS2_GEOMETRY_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_ROS2_GEOMETRY_MSG_CONVERSIONS_H_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <wavemap/core/common.h>

// NOTE: ROS1 routed these through eigen_conversions (tf::pointMsgToEigen and
//       friends). The ROS2 equivalent would be tf2_eigen, but every one of
//       these is a three-field copy, and tf2_eigen's toMsg() overloads are
//       ambiguous between Point and Vector3 for the same Eigen::Vector3d.
//       Writing them out drops a dependency and removes that ambiguity.
namespace wavemap::convert {
inline Point3D pointMsgToPoint3D(const geometry_msgs::msg::Point& msg) {
  return {static_cast<FloatingPoint>(msg.x), static_cast<FloatingPoint>(msg.y),
          static_cast<FloatingPoint>(msg.z)};
}

inline geometry_msgs::msg::Point point3DToPointMsg(const Point3D& point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

inline Point3D point32MsgToPoint3D(const geometry_msgs::msg::Point32& msg) {
  return {msg.x, msg.y, msg.z};
}

inline geometry_msgs::msg::Point32 point3DToPoint32Msg(const Point3D& point) {
  geometry_msgs::msg::Point32 msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

inline Vector3D vector3MsgToVector3D(const geometry_msgs::msg::Vector3& msg) {
  return {static_cast<FloatingPoint>(msg.x), static_cast<FloatingPoint>(msg.y),
          static_cast<FloatingPoint>(msg.z)};
}

inline geometry_msgs::msg::Vector3 vector3DToVector3Msg(
    const Vector3D& vector) {
  geometry_msgs::msg::Vector3 msg;
  msg.x = vector.x();
  msg.y = vector.y();
  msg.z = vector.z();
  return msg;
}

inline Rotation3D quaternionMsgToRotation3D(
    const geometry_msgs::msg::Quaternion& msg) {
  // NOTE: Eigen's quaternion constructor takes (w, x, y, z), not (x, y, z, w).
  return Rotation3D{Eigen::Quaternion<FloatingPoint>{
      static_cast<FloatingPoint>(msg.w), static_cast<FloatingPoint>(msg.x),
      static_cast<FloatingPoint>(msg.y), static_cast<FloatingPoint>(msg.z)}};
}

inline Transformation3D transformMsgToTransformation3D(
    const geometry_msgs::msg::Transform& msg) {
  return Transformation3D{quaternionMsgToRotation3D(msg.rotation),
                          vector3MsgToVector3D(msg.translation)};
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_ROS2_GEOMETRY_MSG_CONVERSIONS_H_
