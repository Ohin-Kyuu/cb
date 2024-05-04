#!/bin/bash

blue_serial_no="215322075868"
yellow_serial_no="215222079970"

blue_tf_info="-0.225 1.13 1.44 -143 3 -160 map realsense_camera 10"
yellow_tf_info="0.225 1.13 1.44 -143 3 160 map realsense_camera 10"

run_script_with_delay() {
    script_path=$1
    delay=$2

    "$script_path" & sleep "$delay"
}

#Reset launch files
echo "Resetting launch files ..."
cp /home/extraction-ws/src/yolo/launch/tf_camera.launch.backup /home/extraction-ws/src/yolo/launch/tf_camera.launch
sleep 3

#BLUE or Yellow team
if [[ $1 == "--blue" ]]; then
    echo -e "Setting up \e[1mBLUE\e[0m team ..."
            
    sed -i "s/args=\"tf_info\"/args=\"$blue_tf_info\"/g" \
            /home/extraction-ws/src/yolo/launch/tf_camera.launch 
elif [[ $1 == "--yellow" ]]; then
    echo -e "Setting up \e[1mYELLOW\e[0m team ..."
            
    sed -i "s/args=\"tf_info\"/args=\"$yellow_tf_info\"/g" \
            /home/extraction-ws/src/yolo/launch/tf_camera.launch 
else
    echo "Usage: $0 [--blue | --yellow]"
    exit 1
fi
sleep 3

chmod +x /home/startup/task/*.sh

echo -e "Starting \e[1mRealsense\e[0m node ..."
sleep 1
run_script_with_delay "/home/startup/task/cb-cam.sh" 30

echo -e "Starting \e[1mTF\e[0m node ..."
sleep 1
run_script_with_delay "/home/startup/task/cb-tf.sh" 10

echo -e "Starting \e[1mYOLO\e[0m node ..."

wait
echo "All scheduled scripts have completed."