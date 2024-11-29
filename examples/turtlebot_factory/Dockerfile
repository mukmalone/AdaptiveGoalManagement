FROM osrf/ros:noetic-desktop-full

# Update and Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    ros-noetic-move-base-msgs \
    libcurl4-openssl-dev

# Setup catkin workspace
WORKDIR /catkin_ws
RUN mkdir -p src && \
    cd src && \
    git clone https://github.com/mukmalone/agm_comm.git && \
    git clone https://github.com/mukmalone/agm_msgs.git && \
    git clone https://github.com/mukmalone/turtle_factory.git

# Build the ROS workspace
# Note: We need to source the setup file to make catkin_make available
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'