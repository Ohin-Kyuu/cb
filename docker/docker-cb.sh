#!/bin/bash
docker compose -p vision-cb down --volumes --remove-orphans

# find /home/vision/pomelo925/eurobot-2024-vision-main/onboard/inspection -type f -name "*.sh" -exec chmod +x {} \;

export DISPLAY=:0.0
xhost +

docker compose -p vision-cb -f compose-build.yml up
docker compose -p vision-cb down --volumes --remove-orphans

docker compose -p vision-cb -f compose-run-cb.yml up -d