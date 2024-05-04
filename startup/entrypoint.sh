#!/bin/bash

blue_serial_no="215322075868"
yellow_serial_no="215222079970"

#Reset launch files
echo "Resetting launch files ..."
cp /home/realsense-ws/src/realsense-ros/realsense2_camera/launch/rs_camera.launch.backup /home/realsense-ws/src/realsense-ros/realsense2_camera/launch/rs_camera.launch
cp /home/realsense-ws/src/yolo/launch/tf_camera.launch.backup /home/realsense-ws/src/yolo/launch/tf_camera.launch
sleep 3

#BLUE or Yellow team
if [[ $1 == "--blue" ]]; then
    echo -e "Setting up \e[1mBLUE\e[0m team ..."

    sed -i "s/default=\"serial_no\"/default=\"$blue_serial_no\"/g" \
            /home/realsense-ws/src/realsense-ros/realsense2_camera/launch/rs_camera.launch

    sed -i 's/args="tf_info"/args="-0.225 1.13 1.44 -143 3 -160 map realsense_camera 10"/g' \
            /home/realsense-ws/src/yolo/launch/tf_camera.launch 
elif [[ $1 == "--yellow" ]]; then
    echo -e "Setting up \e[1mYELLoW\e[0m team ..."

    sed -i "s/default=\"serial_no\"/default=\"$yellow_serial_no\"/g" \
            /home/realsense-ws/src/realsense-ros/realsense2_camera/launch/rs_camera.launch
            
    sed -i 's/args="tf_info"/args="0.225 1.13 1.44 -143 3 160 map realsense_camera 10"/g' \
            /home/realsense-ws/src/yolo/launch/tf_camera.launch 
else
    echo "Usage: $0 [--blue | --yellow]"
    exit 1
fi
sleep 1

# Working directory
cd /home/realsense-ws
source devel/setup.bash

#run CB nodes
echo -e "Starting \e[1mRealsense\e[0m node ..."
sleep 1
roslaunch realsense2_camera rs_camera.launch &
sleep 20

echo -e "Starting \e[1mYOLO\e[0m node ..."
sleep 1
roslaunch yolo tf_camera.launch &
sleep 3
rosrun yolo LTS.py

wait
echo -e "\e[1mCompletely Exit!\e[0m"