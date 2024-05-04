#Base Image
FROM ohin112/l4t-ubuntu20-yolo:ultralytics-ros_noetic 

LABEL org.opencontainers.image.authors="ohin"

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip \
    python3-rosdep

WORKDIR /home/realsense-ws/src
RUN git clone --branch kinetic-devel https://github.com/pal-robotics/ddynamic_reconfigure.git && \
    git clone --branch ros1-legacy --single-branch https://github.com/IntelRealSense/realsense-ros.git && \
    (cd realsense-ros && git checkout $(git tag | sort -V | grep -P "^2.\\d+\\.\\d+" | tail -1)) && \
    rm -rf .git

### install and build dependencies
RUN apt-get update && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    cd .. && \
    /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make' && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && sudo rm -rf /var/lib/apt/lists/* 

ENV SHELL /bin/bash
SHELL ["/bin/bash","-ic"]