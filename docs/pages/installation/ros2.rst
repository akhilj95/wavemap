ROS2 (colcon)
#############
.. highlight:: bash
.. rstcheck: ignore-directives=tab-set-code
.. rstcheck: ignore-roles=gh_file

Wavemap's ROS2 packages target `ROS 2 Jazzy <https://docs.ros.org/en/jazzy/Installation.html>`_ on Ubuntu 24.04 (Noble).
They mirror the ROS1 packages: the same server and rosbag processor, the same topic and service names, and the same YAML config files, which can be used unmodified.

.. note::

    The ROS2 interface does not yet include the launch files for the online sensor setups.

As in ROS1, the launch files start Rviz2 with wavemap's map display by default.
On machines without a display, such as a server or a Docker container without X forwarding, pass ``show_rviz:=false``.

.. _installation-ros2-docker:

Docker
******

If you have not yet installed Docker on your computer, please follow `these instructions <https://docs.docker.com/engine/install/>`_. We also recommend executing the `post-installation steps for Linux <https://docs.docker.com/engine/install/linux-postinstall/>`_, to make Docker available without ``sudo`` priviliges.

The ROS2 image is built from a local copy of the repository::

    git clone https://github.com/ethz-asl/wavemap.git
    cd wavemap
    docker build --target=workspace-built --tag=wavemap_ros2 -f tooling/docker/ros2/full.Dockerfile .

The ``--target=workspace-built`` argument is required. Without it, Docker builds the file's last stage, which only holds the compiler cache.

The resulting image contains a built colcon workspace in ``/home/ci/ros2_ws``. Source it before running any of the commands below::

    source /home/ci/ros2_ws/install/setup.bash

Native install
**************
Install ROS 2 Jazzy by following the `standard instructions <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>`_. The ``ros-jazzy-ros-base`` variant is sufficient.

Make sure these required dependencies are installed::

    sudo apt update
    sudo apt install git build-essential  # General
    sudo apt install python3-rosdep python3-colcon-common-extensions  # ROS

Next, clone the code for wavemap. We recommend using `SSH <https://docs.github.com/en/authentication/connecting-to-github-with-ssh>`_. Alternatively, HTTPS can be used without requiring keys to be set up.

.. tab-set-code::

    .. code-block:: SSH
      :class: no-header

      cd ~
      git clone git@github.com:ethz-asl/wavemap.git

    .. code-block:: HTTPS
      :class: no-header

      cd ~
      git clone https://github.com/ethz-asl/wavemap.git

Unlike for ROS1, the repository must not be cloned into the workspace's ``src`` folder directly.
Its top-level ``CMakeLists.txt`` would make colcon treat the whole repository as a single package and skip the ROS2 packages inside it.
Instead, create a colcon workspace and link only the ROS2 packages into it::

    mkdir -p ~/ros2_ws/src
    ln -s ~/wavemap/interfaces/ros2 ~/ros2_ws/src/wavemap

Make sure rosdep is initialized::

    sudo rosdep init

Then install the remaining system dependencies using::

    source /opt/ros/jazzy/setup.bash
    rosdep update
    rosdep install -y --from-paths ~/ros2_ws/src --ignore-src

Build all of wavemap's ROS2 packages with::

    cd ~/ros2_ws
    colcon build --packages-up-to wavemap_all_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release

Finally, source the workspace::

    source ~/ros2_ws/install/setup.bash

Running wavemap
***************
All of the example configs from the ROS1 package are installed unmodified, in ``$(ros2 pkg prefix wavemap_ros_ros2)/share/wavemap_ros_ros2/config``.
Your own ROS1 configs can be used as they are.
Unlike in ROS1, the config file must always be passed explicitly with ``param_file``.

To run the wavemap server on live data, use::

    ros2 launch wavemap_ros_ros2 wavemap_server.launch.xml param_file:=/path/to/your_config.yaml

To process one or more rosbags as fast as possible, use::

    ros2 launch wavemap_ros_ros2 rosbag_processor.launch.xml param_file:=/path/to/your_config.yaml rosbag_path:="/path/to/first_bag /path/to/second_bag"

As in ROS1, relative rosbag paths are resolved with respect to ``ROS_HOME`` (default ``~/.ros``).
As in ROS1, the rosbag processor stays alive once it is done, so the map can be inspected in Rviz2, unless ``show_rviz:=false`` is set, in which case it exits.
While wavemap is running, its map can be saved with::

    ros2 service call /wavemap/save_map wavemap_msgs_ros2/srv/FilePath "{file_path: /path/to/map.wvmp}"

To run the Panoptic Mapping flat dataset demo, download and extract the `flat dataset <https://doi.org/10.3929/ethz-c-000788335>`_, and run::

    ros2 launch wavemap_ros_ros2 panoptic_mapping_rgbd_flat.launch.xml base_path:=/path/to/flat_dataset/run1

To run one of the Newer College dataset demos (cloister, math, mine or park), convert its sensor and odometry rosbags as described below, and run e.g.::

    ros2 launch wavemap_ros_ros2 newer_college_os0_cloister.launch.xml rosbag_dir:=/path/to/converted/cloister

By default these process the bags in batch mode, using the odometry bag to resolve LiDAR poses. In batch mode, the fixed transforms the sensor rig normally supplies live are instead baked into a short-lived rosbag spanning the input bags' time range, since the batch processor takes TF only from the bags it reads, not from the ROS graph.

Using ROS1 rosbags
******************
ROS2 cannot read ROS1 rosbags directly. They can be converted with the `rosbags <https://ternaris.gitlab.io/rosbags/>`_ Python package::

    sudo apt install python3-venv
    python3 -m venv ~/rosbags_venv
    ~/rosbags_venv/bin/pip install rosbags
    ~/rosbags_venv/bin/rosbags-convert --src your_bag.bag --dst your_bag

The resulting ``your_bag`` directory can be passed to ``rosbag_path`` like any other ROS2 bag.

.. note::

    When a converted bag is replayed live with ``ros2 bag play``, rather than processed with the rosbag processor, check that its ``/tf_static`` topic is recorded with ``durability: transient_local``, under ``offered_qos_profiles`` in the bag's ``metadata.yaml``.
    Otherwise, wavemap never receives these transforms.
    Instead, it reports that frames such as ``odom`` do not exist and skips every input.
    This can be fixed at replay time with ``ros2 bag play --qos-profile-overrides-path``.
