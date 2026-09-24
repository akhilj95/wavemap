#!/usr/bin/env python3

# All credits go to:
# https://github.com/ethz-asl/panoptic_mapping
#
# ROS2 port of interfaces/ros1/wavemap_ros/scripts/
# panoptic_mapping_flat_data_player.py.

import os
import csv

from copy import deepcopy
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
from PIL import Image as PilImage
import numpy as np
from tf2_ros import TransformBroadcaster

from std_srvs.srv import Empty


def quaternion_from_matrix(matrix):
    """Return the (x, y, z, w) quaternion of a 4x4 homogeneous transform.

    Stands in for ROS1's tf.transformations.quaternion_from_matrix, which has
    no ROS2 counterpart.
    """
    m = np.asarray(matrix, dtype=np.float64)[:3, :3]
    trace = np.trace(m)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


class FlatDataPlayer(Node):
    # pylint: disable=too-many-instance-attributes
    def __init__(self):
        """  Initialize ros node and read params """
        super().__init__('flat_data_player')

        # params
        # NOTE: Dynamically typed, since ROS2 would otherwise reject e.g. an
        #       integer play_rate:=2 that ROS1 accepted.
        def param(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(dynamic_typing=True)).value

        self.data_path = str(param('data_path', ''))
        self.global_frame_name = str(param('global_frame_name', 'world'))
        self.sensor_frame_name = str(param('sensor_frame_name', "depth_cam"))
        self.play_rate = float(param('play_rate', 1.0))
        self.wait = str(param('wait', False)).lower() == 'true'
        self.max_frames = int(param('max_frames', 1000000000))
        self.refresh_rate = 100  # Hz
        self.done = False

        # ROS
        self.color_pub = self.create_publisher(Image, "~/color_image", 100)
        self.color_info_pub = self.create_publisher(
            CameraInfo, "~/color_image/camera_info", 100)
        self.depth_pub = self.create_publisher(Image, "~/depth_image", 100)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, "~/depth_image/camera_info", 100)
        self.id_pub = self.create_publisher(Image, "~/id_image", 100)
        self.pose_pub = self.create_publisher(PoseStamped, "~/pose", 100)
        self.tf_broadcaster = TransformBroadcaster(self)

        # setup
        self.cv_bridge = CvBridge()
        stamps_file = os.path.join(self.data_path, 'timestamps.csv')
        self.times = []
        self.ids = []
        self.current_index = 0  # Used to iterate through
        if not os.path.isfile(stamps_file):
            self.get_logger().fatal(
                f"No timestamp file '{stamps_file}' found.")
        with open(stamps_file, 'r') as read_obj:
            csv_reader = csv.reader(read_obj)
            for row in csv_reader:
                if row[0] == "ImageID":
                    continue
                self.ids.append(str(row[0]))
                self.times.append(float(row[1]) / 1e9)

        # Populate the camera_info messages, written out in intrinsics.txt
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = 640
        self.camera_info_msg.height = 480
        self.camera_info_msg.k[0] = 320.0  # fx
        self.camera_info_msg.k[4] = 320.0  # fy
        self.camera_info_msg.k[2] = 320.0  # cx
        self.camera_info_msg.k[5] = 240.0  # cy

        self.ids = [x for _, x in sorted(zip(self.times, self.ids))]
        self.times = sorted(self.times)
        self.times = [(x - self.times[0]) / self.play_rate for x in self.times]
        self.start_time = None

        if self.wait:
            self.start_srv = self.create_service(Empty, '~/start', self.start)
        else:
            self.start(None, None)

    def start(self, _, response):
        self.running = True
        self.timer = self.create_timer(1.0 / self.refresh_rate, self.callback)
        return response

    def shutdown(self, reason):
        # Replaces rospy.signal_shutdown(); main() stops spinning on this.
        self.get_logger().info(reason)
        self.done = True

    def callback(self):
        # Check we should be publishing.
        if not self.running or self.done:
            return

        # Check we're not done.
        if self.current_index >= len(self.times):
            self.get_logger().info("Finished playing the dataset.")
            self.shutdown("Finished playing the dataset.")
            return

        # Check the time.
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        if self.times[self.current_index] > \
                (now - self.start_time).nanoseconds * 1e-9:
            return
        stamp = now.to_msg()

        # Get all data and publish.
        file_id = os.path.join(self.data_path, self.ids[self.current_index])

        # Color.
        color_file = file_id + "_color.png"
        depth_file = file_id + "_depth.tiff"
        pose_file = file_id + "_pose.txt"
        files = [color_file, depth_file, pose_file]
        for f in files:
            if not os.path.isfile(f):
                self.get_logger().warn(
                    f"Could not find file '{f}', skipping frame.")
                self.current_index += 1
                return

        # Load and publish Color image.
        cv_img = cv2.imread(color_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.sensor_frame_name
        self.color_pub.publish(img_msg)

        # Load and publish ID image.
        img_msg = self.cv_bridge.cv2_to_imgmsg(
            np.ascontiguousarray(cv_img[:, :, 0]), "8UC1")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.sensor_frame_name
        self.id_pub.publish(img_msg)

        # Load and publish depth image. These are optional.
        cv_img = PilImage.open(depth_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(np.array(cv_img), "32FC1")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.sensor_frame_name
        self.depth_pub.publish(img_msg)

        # Publish the camera info messages
        cam_info_msg = CameraInfo()
        cam_info_msg = deepcopy(self.camera_info_msg)
        cam_info_msg.header = img_msg.header
        self.depth_info_pub.publish(cam_info_msg)
        self.color_info_pub.publish(cam_info_msg)

        # Load and publish transform.
        if os.path.isfile(pose_file):
            with open(pose_file, 'r') as f:
                pose_data = [float(x) for x in f.read().split()]
                transform = np.eye(4)
                for row in range(4):
                    for col in range(4):
                        transform[row, col] = pose_data[row * 4 + col]
                rotation = quaternion_from_matrix(transform)
                tf_msg = TransformStamped()
                tf_msg.header.stamp = stamp
                tf_msg.header.frame_id = self.global_frame_name
                tf_msg.child_frame_id = self.sensor_frame_name
                tf_msg.transform.translation.x = transform[0, 3]
                tf_msg.transform.translation.y = transform[1, 3]
                tf_msg.transform.translation.z = transform[2, 3]
                tf_msg.transform.rotation.x = rotation[0]
                tf_msg.transform.rotation.y = rotation[1]
                tf_msg.transform.rotation.z = rotation[2]
                tf_msg.transform.rotation.w = rotation[3]
                self.tf_broadcaster.sendTransform(tf_msg)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.global_frame_name
        pose_msg.pose.position.x = pose_data[3]
        pose_msg.pose.position.y = pose_data[7]
        pose_msg.pose.position.z = pose_data[11]
        pose_msg.pose.orientation.x = rotation[0]
        pose_msg.pose.orientation.y = rotation[1]
        pose_msg.pose.orientation.z = rotation[2]
        pose_msg.pose.orientation.w = rotation[3]
        self.pose_pub.publish(pose_msg)

        self.current_index += 1
        if self.current_index > self.max_frames:
            self.shutdown(f"Played reached max frames ({self.max_frames})")


def main():
    rclpy.init()
    flat_data_player = FlatDataPlayer()
    try:
        while rclpy.ok() and not flat_data_player.done:
            rclpy.spin_once(flat_data_player, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    flat_data_player.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
