# Odometry

Wavemap does not estimate the sensor's motion itself.
It looks up the pose of each measurement in TF, so an odometry system, such as a LiDAR-inertial odometry, must publish the sensor's poses to TF while wavemap runs.

## What wavemap needs

* **A TF chain** from wavemap's `world_frame` to the frame of each input. By default this is the frame in the input's message headers (`os_sensor` for Ouster LiDARs), unless the input's `sensor_frame_id` is set. Set `world_frame` to your odometry's fixed frame, and publish the static transforms between your odometry's moving frame and the sensor frame, using the same extrinsics as your odometry.
* **Poses that arrive in time.** Wavemap keeps each measurement queued until its poses are in TF, but only for up to `max_wait_for_pose` (1 s by default), measured against the newer measurements that arrive meanwhile. After that, it drops the measurement and logs a `Max waiting time ... exceeded` warning, so odometry that lags further behind the sensor loses measurements.
* **High-rate poses for motion undistortion.** With `undistort_motion: true`, wavemap corrects each LiDAR scan for the sensor's motion during the scan, using poses at several times across the scan. This gives the sharpest maps with high-rate poses, such as IMU-rate poses from a LiDAR-inertial odometry.

Until the odometry publishes its first pose, wavemap skips the incoming measurements and logs a warning for each of them.
This is expected.
With motion undistortion, a scan whose start pose is not in TF is skipped, while a scan whose end pose has not arrived yet is retried.

## iG-LIO

We tested wavemap with the ROS2 port of [iG-LIO](https://github.com/Forestry-Robotics-UC/ig_lio/tree/ros2-jazzy), running live on an Ouster LiDAR and its built-in IMU.
Wavemap's maps with it match those built with the ROS1 version of iG-LIO and wavemap.
The following points are needed to connect the two:

* **TF chain**: iG-LIO publishes the transform from its `odom_frame` (default `odom`) to its `base_frame` (default `base_link`), where the base frame is the IMU. Set wavemap's `world_frame` to iG-LIO's `odom_frame`.

* **Motion undistortion**: `undistort_motion: true` works with iG-LIO's poses.

* **Initialization**: iG-LIO estimates its IMU biases and the gravity direction from the first `init_count` IMU measurements (20 by default, i.e. 0.2 s at 100 Hz). If the sensor moves during this period, iG-LIO diverges, and so does wavemap's map. Keep the sensor still when iG-LIO starts, or raise `init_count`.

* **Transport**: iG-LIO receives the same large pointclouds as wavemap, so it needs the same [DDS settings](dds.md). With Fast DDS's default shared-memory transport, it silently lost about 20% of an Ouster's pointclouds in our tests. When wavemap runs in its Docker image, which uses Cyclone DDS, or iG-LIO runs in a separate Docker container from wavemap, see [containers and multiple computers](dds.md#containers-and-multiple-computers).
