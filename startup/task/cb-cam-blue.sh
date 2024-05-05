#!/bin/bash

# Description: This script is used to start a RealSense camera with specified settings including optional JSON configuration.
# Usage: ./script_name.sh [-c camera_name] [-s serial_no] [-f] [-j path_to_json]

### Default settings
camera="cbcam"
serial_no="215322075868"

# Parse command line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -c|--camera) camera="$2"; shift ;; # Set camera name
        -s|--serial-no) serial_no="$2"; shift ;; # Set camera serial number
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

echo -e "Starting \e[1mRealsense\e[0m node ..."
sleep 1

cd /home/realsense-ws
source devel/setup.bash

roslaunch realsense2_camera rs_camera.launch \
    camera:=$camera \
    serial_no:=$serial_no \

wait
