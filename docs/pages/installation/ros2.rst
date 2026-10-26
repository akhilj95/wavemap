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
    docker build --target=workspace-built --tag=wavemap2 -f tooling/docker/ros2/full.Dockerfile .

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
    colcon build --packages-up-to wavemap2_all --cmake-args -DCMAKE_BUILD_TYPE=Release

Finally, source the workspace::

    source ~/ros2_ws/install/setup.bash

Running wavemap
***************
All of the example configs from the ROS1 package are installed unmodified, in ``$(ros2 pkg prefix wavemap2_ros)/share/wavemap2_ros/config``.
Your own ROS1 configs can be used as they are.
Unlike in ROS1, the config file must always be passed explicitly with ``param_file``.

To run the wavemap server on live data, use::

    ros2 launch wavemap2_ros wavemap_server.launch.xml param_file:=/path/to/your_config.yaml

To process one or more rosbags as fast as possible, use::

    ros2 launch wavemap2_ros rosbag_processor.launch.xml param_file:=/path/to/your_config.yaml rosbag_path:="/path/to/first_bag /path/to/second_bag"

As in ROS1, relative rosbag paths are resolved with respect to ``ROS_HOME`` (default ``~/.ros``).
As in ROS1, the rosbag processor stays alive once it is done, so the map can be inspected in Rviz2, unless ``show_rviz:=false`` is set, in which case it exits.
While wavemap is running, its map can be saved with::

    ros2 service call /wavemap/save_map wavemap2_msgs/srv/FilePath "{file_path: /path/to/map.wvmp}"

To run the Panoptic Mapping flat dataset demo, download and extract the `flat dataset <https://doi.org/10.3929/ethz-c-000788335>`_, and run::

    ros2 launch wavemap2_ros panoptic_mapping_rgbd_flat.launch.xml base_path:=/path/to/flat_dataset/run1

To run one of the Newer College dataset demos (cloister, math, mine or park), convert its sensor and odometry rosbags as described below, and run e.g.::

    ros2 launch wavemap2_ros newer_college_os0_cloister.launch.xml rosbag_dir:=/path/to/converted/cloister

By default these process the bags in batch mode, using the odometry bag to resolve LiDAR poses. In batch mode, the fixed transforms the sensor rig normally supplies live are instead published by ``static_transform_publisher`` nodes started alongside the rosbag processor, and the bags' own ``/tf``/``/tf_static`` are republished onto the ROS graph, since the batch processor takes TF from a live ``TransformListener`` rather than reading it out of the bags directly.

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

.. _installation-ros2-large-pointclouds:

Large pointclouds over DDS
**************************
When pointclouds arrive over a ROS2 topic, rather than through the rosbag processor, two settings are needed to receive large clouds, such as an Ouster's, without silently losing most of them.

First, ``wavemap_server.launch.xml`` sets ``FASTDDS_BUILTIN_TRANSPORTS=UDPv4`` on the ``wavemap`` node, because Fast DDS's default shared-memory transport silently drops most of a large best-effort topic once the node falls behind.
If you run ``ros_server`` directly, or write your own launch files around it instead of including ``wavemap_server.launch.xml``, set this environment variable yourself.

Second, over UDP, the kernel's receive buffer must be large enough to hold a whole pointcloud.
Fast DDS does not choose a buffer size for its sockets, so they get the system default, ``net.core.rmem_default``, which is about 208 KB on Ubuntu.
Raise the limit and the default with::

    sudo sysctl -w net.core.rmem_max=67108864 net.core.rmem_default=67108864

When running wavemap in Docker, set them on the host.

.. _installation-ros2-map-msg-layout:

Map message layout
******************
Wavemap's map messages can be built in one of two layouts, selected with the ``WAVEMAP2_MSGS_LAYOUT`` CMake option of the ``wavemap2_msgs`` package:

* ``flat`` (default): Each octree's nodes are sent as two arrays of numbers, ``node_detail_coefficients`` and ``node_allocated_children_bitsets``, which DDS serializes in one go.
* ``nested``: Each octree's nodes are sent as an array of ``WaveletOctreeNode`` messages, exactly as in wavemap's ROS1 messages. Some DDS implementations serialize such arrays one element at a time. In our tests, Cyclone DDS took about 80 ms to publish a 30 MB map in this layout, compared to about 10 ms in the flat layout, and the server's mapping is paused while it publishes.

Both layouts use the same message names and carry the same map.
Choose ``nested`` if your map messages need to match ROS1's, for example to bridge them to ROS1, to read map topics from rosbags recorded with the nested layout, or because your own code reads the messages' ``nodes`` fields.

To build with the nested layout, pass the option to colcon::

    colcon build --packages-up-to wavemap2_all --cmake-args -DCMAKE_BUILD_TYPE=Release -DWAVEMAP2_MSGS_LAYOUT=nested

For the :ref:`Docker image <installation-ros2-docker>`, add ``--build-arg WAVEMAP2_MSGS_LAYOUT=nested`` to the ``docker build`` command instead.

CMake remembers the option, so later builds of the same workspace keep the last layout that was passed, even without ``--cmake-args``.
When the layout of an existing workspace is switched, colcon rebuilds the packages that depend on the messages.
Switching from nested to flat, however, leaves the nested layout's ``WaveletOctreeNode`` files behind in the ``install`` folder, so we recommend deleting the ``build`` and ``install`` folders before switching.

Every node that exchanges maps with wavemap, including Rviz2 with wavemap's plugin, must be built with the same layout as the server.
If the layouts differ, the subscriber still connects to the map topic, but cannot read the maps:

* With Fast DDS, it receives no maps at all. Depending on which side uses which layout, Fast DDS prints ``sequence size exceeds remaining buffer`` or nothing.
* With Cyclone DDS, the maps arrive, but ``rosMsgToMap`` rejects them and logs a warning that names the subscriber's layout. This includes wavemap's Rviz2 plugin.

In both cases, ``ros2 topic info -v /wavemap/map`` lists a different ``Topic type hash`` for the publisher than for the subscriptions.
When it starts, the server logs the layout it publishes in::

    Publishing maps on '/wavemap/map' using the 'flat' map message layout (WAVEMAP2_MSGS_LAYOUT).

To check which layout a workspace was built with, run::

    ros2 interface show wavemap2_msgs/msg/HashedWaveletOctreeBlock

The flat layout lists ``node_detail_coefficients`` and ``node_allocated_children_bitsets``, while the nested layout lists ``WaveletOctreeNode[] nodes``.
Query ``HashedWaveletOctreeBlock`` rather than ``WaveletOctree``, since ROS 2 Jazzy's ``ros2 interface show`` prints the definition of ``HashedWaveletOctree`` when asked for ``WaveletOctree``.

To check which layout a rosbag's map topic was recorded with, compare its ``type_description_hash`` in the bag's ``metadata.yaml`` with the ``hash_string`` of ``wavemap2_msgs/msg/Map`` in ``$(ros2 pkg prefix wavemap2_msgs)/share/wavemap2_msgs/msg/Map.json``.
If they differ, the bag was recorded with the other layout, or with a version of wavemap whose map messages differ.
