## SetUp


## Minimum Hardware Requirements
We recommend the following PC operating environment for this tournament.


* OS: Ubuntu 22.04
* CPU: Intel Corei7 (8 cores) or higher
* GPU: NVIDIA Geforce RTX 3080 (VRAM 12 GB) or higher
* Memory: 32 GB or more
* Storage: SSD 30 GB or higher


If you are unable to prepare a PC that meets the above specifications, please refer to the "For participants with two PCs" below.
#### For participants with two PCs
#### Autoware PC
* OS: Ubuntu 20.04
* CPU: Intel Corei7 (8 cores) or higher
* GPU: NVIDIA Geforce GTX 1080 or higher
* Memory: 16 GB or higher
* Storage: SSD 10 GB or higher
* For more information [here](https://autowarefoundation.github.io/autoware-documentation/main/installation/)


#### AWSIM PC
* OS: Ubuntu 22.04 or Windows 10/11
* CPU: Intel Corei7 (6 cores and 12 threads) or higher
* GPU: NVIDIA Geforce RTX 2080 Ti or higher
* For more information [click here](https://tier4.github.io/AWSIM/)


Autoware and AWSIM PCs should be located in the same network.
If they are located in the same network, topic communication between PCs is basically possible without any additional settings. In the unlikely event that topic communication is not possible, please disable the firewall or review the rules.
  
    
## Environment Setup
### AWSIM(Ubuntu)
#### Preparation
* Install Nvidia drivers
  1. add repository
  ```
  sudo add-apt-repository ppa:graphics-drivers/ppa
  ```
  2. update package list
  ```
  sudo apt update
  ```
  3. install 
  ```
  sudo ubuntu-drivers autoinstall
  ```
  4. After rebooting, run the following command to confirm that the installation has completed.
  ```
  nvidia-smi
  ```
  ![nvidia-smi](../images/setup/nvidia-smi.png)
 
 * Install Vulkun
    1. update package list
    ```
    sudo apt update
    ```
    2. install libvulkan1
    ```
    sudo apt install libvulkan1
    ```
 * Prepare the course
   1. [Download](https://drive.google.com/file/d/1aduBKhYGI0mhhRbgu4B05pBTyFXcZsGN/view?usp=sharing) the executable file for the competition and unzip it
   2. change permissions as shown in the figure    
   ![Change the permissions as shown in the figure ](../images/setup/permmision.png)  
   Double-click the file to launch it.
   4. confirm that the following screen is displayed
      ![awsim_ubuntu](../images/setup/awsim_ubuntu.png)
        
### AWSIM(Windows)
  1. [Download](https://drive.google.com/file/d/1L6jr9wttxA2aLl8IqC3xDXIuQUfjMTAJ/view?usp=sharing) the executable file for the convention and unzip it. 
  2. double-click the file to start it
  Confirm that the following screen is displayed.
    ![Confirm that a screen similar to the following appears](../images/setup/awsim_win.png)
    
### Autoware
Docker image of Autoware (using CUDA) is available for this competition.
  
* Preparation  
Please install the following.
  * [docker](https://docs.docker.com/engine/install/ubuntu/)
  * [rocker](https://github.com/osrf/rocker) 
     * [docker]() * [rocker]() is used to use GUI such as Rviz and rqt in Docker container.
  * [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
  * [git lfs](https://packagecloud.io/github/git-lfs/install)
  * [ROS2](https://docs.ros.org/en/humble/index.html) (video confirmed version: Humble)
  
* Prepare and launch Docker image - Prepare Autoware
   1. get a Docker image
    ```
   docker pull ghcr.io/automotiveaichallenge/aichallenge2023-sim/autoware-universe-cuda:v1
    ```
    If the above method takes a long time or times out, you can use the following method.  
　We have placed a tarball of the image at [here](https://drive.google.com/file/d/1mOEpiN36UPe70NqiibloDcd_ewgMr_5P/view?usp=sharing). Please use the following command
   ````
   docker load < autoware-universe-cuda_v1.tar.gz
   ```` 
    2. download the data for the competition
    ```
    sudo apt install -y git-lfs
    git lfs clone https://github.com/AutomotiveAIChallenge/aichallenge2023-sim
    ```
    3. start rocker
    ````
    cd . /aichallenge2023-sim
    rocker --nvidia --x11 --user --net host --privileged --volume autoware:/aichallenge -- ghcr.io/automotiveaichallenge/aichallenge2023-sim/ autoware-universe-cuda:v1
    ```
      
 * Verify Autoware operation  
   This section describes how to check the operation of Autoware using AWSIM. 
   1. Start AWSIM. 
   2. Start Autoware.
   ````
   # In the Rocker container
	cd /aichallenge/aichallenge_ws
	colcon build 
	source install/setup.bash
	cd /aichallenge
	ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=mapfile
   ```` 
   3. Confirm that the following screen (Rviz2) is displayed.  
![autoware1](../images/setup/autoware1.png)   
     
   4. Open ``add new Panel`` from Panel in Rviz tab and add AutowareStatePanel.  
   ![autoware2](../images/setup/autoware2.png)   
   ![autoware3](../images/setup/autoware2.png) 
     
    5. Confirmation of self-location estimation  
    ![autoware4](../images/setup/autoware4.png) 
      
    6. If it is not estimated correctly, select 2D Pose Estimate in the tab, and drag the actual position of the vehicle.  
    ![autoware5](../images/setup/autoware5.png)       
      
    7. Select 2D Goal Pose in the tab and drag to specify the goal position.  
     ![autoware6](../images/setup/autoware6.png)          
       
     8. check that the route is displayed and "waiting for engage" as shown in the image (it takes a little time after specifying the route)
     ![autoware7](../images/setup/autoware7.png) 
       
     9. press the "engage" button and confirm that the automatic operation starts  
     ![autoware8](../images/setup/autoware8.png)   
        