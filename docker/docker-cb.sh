#!/bin/bash
docker compose -p vision-cb down --volumes --remove-orphans

find /home/nano/cb/ -type f -name "*.sh" -exec chmod +x {} \;

export DISPLAY=:0.0
#../system/all_reset.sh

docker compose -p vision-cb -f $HOME/cb/docker/compose-build.yml up
docker compose -p vision-cb down --volumes --remove-orphans

docker compose -p vision-cb -f $HOME/cb/docker/compose-run-cb.yml up