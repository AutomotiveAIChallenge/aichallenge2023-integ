#!/bin/bash

sudo ip link set multicast on lo

source aichallenge_ws/install/setup.bash
rm -f result.json
ros2 launch aichallenge_launch aichallenge.launch.xml
