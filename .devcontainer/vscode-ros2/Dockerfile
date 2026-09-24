ARG VERSION=latest

FROM ghcr.io/ethz-asl/wavemap_ros2:${VERSION}

# Create a non-root user and make it the default user
# NOTE: The ROS2 base image already ships an 'ubuntu' user with UID/GID 1000,
#       which is removed first so that 'ci' can take over that UID.
ARG USERNAME=ci
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if id -u ubuntu > /dev/null 2>&1; then userdel -r ubuntu; fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash $USERNAME && \
    cp -a /etc/skel/.bashrc /home/$USERNAME/ && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi' \
      >> /home/$USERNAME/.bashrc && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

# Build the package
WORKDIR /home/$USERNAME/ros2_ws
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-up-to wavemap_all_ros2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
