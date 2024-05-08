#!/bin/bash

blue_tf_info="-0.225 1.13 1.44 -143 3 -160 map realsense_camera 10"

#Reset launch files
echo "Resetting launch files ..."
cp /home/extraction-ws/src/yolo/launch/tf_camera.launch.backup /home/extraction-ws/src/yolo/launch/tf_camera.launch
sleep 3

sed -i "s/args=\"tf_info\"/args=\"$blue_tf_info\"/g" /home/extraction-ws/src/yolo/launch/tf_camera.launch
wait
echo -e "Setting up \e[1mBLUE\e[0m team ..."
sleep 1

cd /home/extraction-ws
source devel/setup.bash

roslaunch yolo tf_camera.launch 
wait
