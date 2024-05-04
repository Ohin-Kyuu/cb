#Base Image
FROM ohin112/l4t-ubuntu20-yolo:ultralytics-ros_noetic 

LABEL org.opencontainers.image.authors="ohin"

ENV SHELL /bin/bash

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip \
    python3-rosdep

WORKDIR /root/realsense-ws
RUN git clone https://github.com/HungPinChiu/Vison-CB_workspace.git src && \
    apt update && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make' && \
    rm -rf /var/lib/apt/lists/* 

SHELL ["/bin/bash","-ic"]



