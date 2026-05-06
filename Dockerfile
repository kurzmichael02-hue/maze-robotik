FROM ros2:latest

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# was im base-image fehlt: slam, nav2, ein paar tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rviz2 \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-stereo-image-proc \
    ros-jazzy-rqt-image-view \
    python3-pip \
    python3-colcon-common-extensions \
    nano \
    && rm -rf /var/lib/apt/lists/*

# auto-source ros + workspace beim bash-start
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo 'if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi' >> ~/.bashrc \
    && echo 'export GZ_SIM_RESOURCE_PATH=/root/ros2_ws/src' >> ~/.bashrc

WORKDIR /root/ros2_ws

CMD ["/bin/bash"]
