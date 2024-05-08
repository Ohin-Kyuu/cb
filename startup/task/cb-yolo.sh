#!/bin/bash

cd /home/extraction-ws
source devel/setup.bash

rosrun yolo LTS.py
wait
