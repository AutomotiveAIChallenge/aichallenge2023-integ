# Local Enviroment.
 &emsp; Participants will be asked to create a ROS2 package to carry out the scenario, and the following ROS2 package is provided in aichallenge2023-sim/docker/aichallenge/aichallenge_ws/src as sample code to serve as a base for this in this repository The following ROS2 package is provided.
 &emsp; Please place the ROS2 package you created under aichallenge_ws/src/aichallenge_submit and follow the steps below to build and run it.
  
## Sample Code
 The following is a partial configuration under &emsp;aichallenge2023-sim/docker/aichallenge/aichallenge_ws/src.
* aichallenge_launch
    * Contains the main launch file aichallenge.launch.xml. All ROS2 nodes are launched from this launch file.
* aichallenge_scoring
    * aichallenge_scoring is responsible for the tasks required for scoring and ranking the AI Challenge participants.
* aichallenge_scoring_msgs
    * msgs for scoring
* aichallenge_scoring_result
    * Calculate scores from /aichallenge/score and /aichallenge/collision.
* aichallenge_submit
    * All ROS2 packages implemented by participants should be placed in this directory, as only the contents of this directory will be submitted at the time of submission.
    * Please modify this launch file accordingly to configure the ROS2 nodes you have implemented to be launched. Please modify this launch file accordingly and configure it so that your ROS2 node will be activated.
    * self_driving_controlloer
        * This is a sample of automatic driving.
    * autoware_launch
        * Autoware launch configuration repository. It contains node configurations and their parameters. If you wish to make changes to Autoware's behavior, please check here first.

### Docker Image Build
```
#In the aichallenge2023-sim directory
cd docker
bash build.sh
```

### Docker Container Run
```
#In the aichallenge2023-sim directory
cd docker
bash run_container.sh
```

### Code Build
```
# In the Rocker container
cd /aichallenge
bash build.sh
 ```
 
### Sample Code Run
 ```
# In the Rocker container
cd /aichallenge
bash run.sh
```


 &emsp; At this point, the setup and execution on the Autoware side is complete. If the setup was successful, rviz will display a point cloud map and start automatic operation.