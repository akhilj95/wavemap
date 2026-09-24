ARG FROM_IMAGE=ros:jazzy-ros-base-noble
ARG USER_HOME=/home/ci
ARG REPOSITORY_NAME=wavemap
ARG REPOSITORY_PATH=${USER_HOME}/${REPOSITORY_NAME}
ARG COLCON_WS_PATH=${USER_HOME}/ros2_ws
ARG CCACHE_DIR=${USER_HOME}/ccache
ARG ROS_HOME=${USER_HOME}/.ros
ARG PACKAGE_NAME=wavemap_all_ros2

# NOTE: Unlike the ROS1 image, the repository does not live inside the
#       workspace's src/ folder. The repo root's CMakeLists.txt would make
#       colcon identify the whole repository as a single package and skip
#       everything below it, so only interfaces/ros2 is symlinked into the
#       workspace instead. This is also the layout the installation docs use.

# hadolint ignore=DL3006
FROM $FROM_IMAGE AS source-filter

# Copy in the project's source
ARG REPOSITORY_PATH
WORKDIR $REPOSITORY_PATH
COPY . .

# Cache the manifests of all ROS2 packages for use in subsequent stages
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN mkdir -p /tmp/manifests && \
    find ./interfaces/ros2 -name "package.xml" -exec \
      cp --parents -t /tmp/manifests {} \; && \
    echo "Manifests hash:" && \
    find /tmp/manifests -type f -print0 | sort -z | \
      xargs -0 sha1sum | sha1sum

# hadolint ignore=DL3006
FROM $FROM_IMAGE AS dependency-installer

# Load the ROS2 package manifest files
WORKDIR /tmp/manifests
COPY --from=source-filter /tmp/manifests .

# Install general and ROS-related system dependencies
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_HOME
ENV ROS_HOME=$ROS_HOME
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends git ccache && \
    rosdep update --rosdistro "$ROS_DISTRO" && \
    rosdep install --from-paths interfaces/ros2 --ignore-src -q -y && \
    rm -rf /var/lib/apt/lists/* /tmp/manifests

# Add ccache to the path and set where it stores its cache
ARG CCACHE_DIR
ENV PATH="/usr/lib/ccache:${PATH}" CCACHE_DIR=$CCACHE_DIR


FROM dependency-installer AS workspace

# Load the repository and link its ROS2 packages into the colcon workspace
ARG REPOSITORY_PATH
ARG COLCON_WS_PATH
COPY --from=source-filter $REPOSITORY_PATH $REPOSITORY_PATH
RUN mkdir -p $COLCON_WS_PATH/src && \
    ln -s $REPOSITORY_PATH/interfaces/ros2 $COLCON_WS_PATH/src/wavemap

# Update the entrypoint to also source the workspace, once it has been built
# hadolint ignore=SC2016
RUN sed --in-place \
      's|^source .*|&\nif [ -f "'$COLCON_WS_PATH'/install/setup.bash" ]; then source "'$COLCON_WS_PATH'/install/setup.bash" --; fi|' \
      /ros_entrypoint.sh && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "if [ -f '$COLCON_WS_PATH'/install/setup.bash ]; then source '$COLCON_WS_PATH'/install/setup.bash; fi" >> ~/.bashrc
WORKDIR $COLCON_WS_PATH


FROM workspace AS workspace-builder

# Build our package
# NOTE: colcon has no persistent workspace configuration equivalent to
#       `catkin config --cmake-args`, so the build type is passed explicitly.
ARG COLCON_WS_PATH
ARG PACKAGE_NAME
WORKDIR $COLCON_WS_PATH
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --packages-up-to $PACKAGE_NAME \
      --event-handlers console_cohesion+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release


FROM workspace AS workspace-built

# Pull in the compiled colcon workspace (but without ccache files etc)
ARG COLCON_WS_PATH
WORKDIR $COLCON_WS_PATH
COPY --from=workspace-builder $COLCON_WS_PATH .


FROM scratch AS workspace-builder-ccache-extractor

# Extract the ccache cache directory from the workspace-builder stage
ARG CCACHE_DIR
WORKDIR /
COPY --from=workspace-builder $CCACHE_DIR .
