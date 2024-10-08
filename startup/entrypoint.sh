#!/bin/bash

run_script_with_delay() {
    script_path=$1
    delay=$2

    "$script_path" & sleep "$delay"
}

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 [--blue | --yellow]"
    exit 1
fi

chmod +x /home/startup/task/*.sh

### STARTUP SEQUENCE ###
if [[ $1 == "--blue" ]]; then
    echo -e "Choosing \e[1mBLUE\e[0m team ..."
    run_script_with_delay "/home/startup/task/cb-cam-blue.sh" 1
    run_script_with_delay "/home/startup/task/cb-tf-blue.sh" 1
elif [[ $1 == "--yellow" ]]; then
    echo -e "Choosing \e[1mYELLOW\e[0m team ..."
    run_script_with_delay "/home/startup/task/cb-cam-yellow.sh" 1
    run_script_with_delay "/home/startup/task/cb-tf-yellow.sh" 1
fi

echo -e "Starting \e[1mYOLO\e[0m node ..."
run_script_with_delay "/home/startup/task/cb-yolo.sh" 1

wait
echo "All scheduled scripts have completed."