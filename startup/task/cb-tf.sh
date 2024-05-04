#!/bin/bash

cd /home/extraction-ws
source devel/setup.bash

roslaunch yolo tf_camera.launch 
wait
