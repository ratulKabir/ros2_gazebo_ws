# Base: full ROS 2 Humble desktop environment with RViz & Gazebo deps
FROM osrf/ros:humble-desktop-full

# Install Gazebo Fortress + bridge
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    gz-fortress \
    && rm -rf /var/lib/apt/lists/*

# Add some useful tools
RUN apt-get update && apt-get install -y git vim nano build-essential

# Setup ROS sourcing automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
