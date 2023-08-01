#!/bin/bash

export PATH="$PATH:/root/.local/bin"
export PATH="/usr/local/cuda/bin:$PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export XDG_RUNTIME_DIR=/tmp/xdg
export ROS_LOCALHOST_ONLY=0

cd /output

source /aichallenge/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.xml &

# Wait result.js
echo "Wait for result.json."
until [ -f /aichallenge/result.json ]
do
  sleep 5
done

echo "Generation of result.json is completed."
cp /aichallenge/result.json /output