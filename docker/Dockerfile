#Base Image
FROM ohin112/l4t-ubuntu20-yolo:ultralytics-ros_noetic 

LABEL org.opencontainers.image.authors="ohin"

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip \
    python3-rosdep

RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra/' >> $HOME/.bashrc

ENV SHELL /bin/bash
SHELL ["/bin/bash","-ic"]



q