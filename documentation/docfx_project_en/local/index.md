# Local Enviroment

<br>

<!-- > [!REGISTER]
> Register from here!
> [https://www.jsae.or.jp/jaaic/en/index.php](https://www.jsae.or.jp/jaaic/en/index.php)

<br> -->

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
    * initialpose_publisher
      * Since the default initial position estimation may result in initial position deviations, this function provides the ability to set the vehicle position at a pre-defined initial position.
    * pose_initializer_custom
      * The pose_initializer package in autoware.universe has been modified to discard the posture calculated by Monte-Carlo and NDT Matching and set it with the posture sent by the `initialpose_publisher`.
    * self_driving_controlloer
        * Provides the ability to set GoalPose and engage required to start automatic operation.
    * autoware_launch
        * We have copied and partially edited the launch, config-related packages of Autoware; Autoware in the Docker image has removed the packages included here. You can modify the behavior of Autoware by editing here.
        * If you want to use the files before the modification, please copy the files from [autoware_launch](https://github.com/autowarefoundation/autoware_launch/tree/awsim-stable), [autoware_universe's launch directory](https://github.com/autowarefoundation/autoware.universe/tree/awsim-stable/launch).

### Steps for Execution

1. Docker Image Build
```
#In the aichallenge2023-sim directory
cd docker
bash build.sh
```

2. Docker Container Run
```
#In the aichallenge2023-sim directory
cd docker
bash run_container.sh
```

3. Code Build
```
# In the Rocker container
cd /aichallenge
bash build.sh
 ```
 4. Start AWSIM  
Start AWSIM by referring to [Setup page](../setup/index.html).

5. Sample Code Run
 ```
# In the Rocker container
cd /aichallenge
bash run.sh
```
 &emsp; If setup is successful, rviz will display a point cloud map and begin automatic operation.
 
 ### Customizing Autoware

 Ways in which Autoware can be customized, or new packages be added into Autoware, are explained in the [Customizing Autoware](../customize/index.html) page.
