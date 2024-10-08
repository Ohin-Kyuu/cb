#!/bin/bash

find $HOME/cb/ -type f -name "*.sh" -exec chmod +x {} \;
#export DISPLAY=:0.0
/bin/bash -ic $HOME/cb/system/all_reset.sh

docker compose -p vision-cb down --volumes --remove-orphans

docker compose -p vision-cb -f $HOME/cb/docker/compose-build.yml up
docker compose -p vision-cb down --volumes --remove-orphans

docker compose -p vision-cb -f $HOME/cb/docker/compose-run-cb-yellow.yml up

# docker exec -it cb_run bash
# rostopic echo /cbcam/objects/six_region_counts
# rostopic echo /robot/objects/global_info