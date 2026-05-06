FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y --no-install-recommends \
    apt-utils \
    x11-apps \
    git \
    nano \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Jazzy + Gazebo Harmonic bridge
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    && rm -rf /var/lib/apt/lists/*

# slam, nav, teleop, xacro
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-stereo-image-proc \
    ros-jazzy-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# bashrc setup
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo 'if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi' >> ~/.bashrc \
    && echo 'export GZ_SIM_RESOURCE_PATH=/root/ros2_ws/src' >> ~/.bashrc

WORKDIR /root/ros2_ws

CMD ["/bin/bash"]
