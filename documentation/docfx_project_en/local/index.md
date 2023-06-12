# Local Enviroment
　The following ROS2 package is provided in autoware/aichallenge_ws/src as a sample code to be used as a base in this repository.
  　Please place the ROS2 package you have created under aichallenge_ws/src/aichallenge_submit and follow the steps below to build and execute it.
## Sample Code
### About Sample Code  
* aichallenge_launch
  * Contains the main launch file aichallenge.launch.xml. All ROS2 nodes are launched from this launch file.
* aichallenge_eval
  * Package for score calculation.
* aichallenge_score_msgs
  * aichallenge_score_msgs * Contains message definitions.
* aichallenge_submit
  * The contents of this directory may be freely modified.
  * All ROS2 packages implemented by participants should be placed in this directory, as only the contents of this directory will be submitted at the time of submission. The following packages are included in the distribution phase.
  * aichallenge_submit_launch
    * aichallenge_submit_launch.launch.xml is called from the main launch file aichallenge.launch.xml, so please modify this launch file accordingly to configure the ROS2 nodes you have implemented to be launched. Please modify this launch file accordingly and configure it to start the ROS2 node you have implemented.
  * sample_code_cpp
    * sample automatic running implementation.
  * obstacle_stop_planner_custom
    * Fix the problem that obstacles are mis-detected from obstacle_stop_planner in autoware.universe.
* tier4_\*_launch
  * Copy and partially edit autoware's launch file. tier4_\*_launch in autoware has been removed, so be sure to leave this one in aichallenge_submit.
The file has been changed to call obstacle_stop_planner_custom instead of obstacle_stop_planner.
  
### Build Sample Code
````
# In the Rocker container
cd /aichallenge/aichallenge_ws
rosdep update
rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build
````

### Run Sample Code
```
# In the Rocker container
source /aichallenge/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.xml
```

At this point, the setup and execution on the Autoware side is complete. If the setup is successful, rviz will display a point cloud map and start automatic operation.