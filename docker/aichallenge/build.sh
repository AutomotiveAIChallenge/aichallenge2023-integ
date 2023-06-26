#!/bin/bash

cd aichallenge_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ..