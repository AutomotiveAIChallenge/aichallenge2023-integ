# FAQ

<br>

## ROS FAQ

<br>

### When creating a package in Python, a "no module named *" error occurs during execution
Please check if the submodule has been added to setup.py.
please refer to [this link](https://zenn.dev/tasada038/articles/5d8ba66aa34b85#setup.py%E3%81%ABsubmodules%E3%81%A8%E3%81%97%E3%81%A6%E3%83%91%E3%83%83%E3%82%B1%E3%83%BC%E3%82%B8%E3%82%92%E8%BF%BD%E5%8A%A0%E3%81%99%E3%82%8B)

### `$ ros2 topic list` is not displayed
- Ensure your machine's `ROS_DOMAIN_ID` matches. (If you haven't set the `ROS_DOMAIN_ID`, there's no issue.)
- Ensure `ROS2` is sourced.

<br>

### Using AWSIM on Windows and Autoware on Ubuntu but `$ ros2 topic list` is not displayed
- Allow communication in the Windows Firewall.
- Execute `ros2 daemon stop` and `ros2 daemon start`, check for any lingering processes, and restart.

<br>

### No path found in Rviz
- Verify that your map data is correct. This includes PointCloud, VectorMap, and 3D fbx models.

<br>

### AWSIM and Autoware network is unstable
When testing locally, setting `ROS_LOCALHOST_ONLY=1` in all terminals can improve communication speed. For this competition, configurations of two PCs (Windows+Linux, Linux+Linux) and a single PC (only Linux) are considered. Please refer to the following settings:
- In the evaluation environment, set `ROS_LOCALHOST_ONLY=0` in [this file](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/blob/main/docker/evaluation/main.bash).
- On the container side, set `ROS_LOCALHOST_ONLY=0` in [this file](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/blob/main/docker/Dockerfile).

If you feel that machine performance or communication speed is insufficient, you can change the execution to be localhost-only as follows:
- Set ROS to be localhost-only. Add the following lines to your `.bashrc`. Note: After OS startup, the password will be prompted at terminal startup, and `sudo ip link set lo multicast on` is required for the first time.
  ```bash
  export ROS_LOCALHOST_ONLY=1
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

  if [ ! -e /tmp/cycloneDDS_configured ]; then
      sudo sysctl -w net.core.rmem_max=2147483647
      sudo ip link set lo multicast on
      touch /tmp/cycloneDDS_configured
  fi
  ```
Note: If you forget to modify as written in `.bashrc`, it will always be applied. Always track changes, such as verifying with `echo $ROS_LOCALHOST_ONLY`. Also, be aware that `ROS_LOCALHOST_ONLY` is specified in the executable file.

<br>

### Launched Autoware is not stable

Consider setting a wait time for Autoware to start.

[Refer here](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/issues/31)

```
<timer period="150.0">
     <include file="$(find-pkg-share self_driving_controller)/launch/self_driving_controller.launch.xml" />
</timer>
```

<br>

### Unable to Launch Rocker

[Refer here](https://github.com/AutomotiveAIChallenge/aichallenge2023-sim/issues/21#issuecomment-1637851299)

<br>

## Setup FAQ

<br>

### Rviz Black Screen when running container

[Refer here](https://github.com/ros2/rviz/issues/948)

```
add-apt-repository ppa:kisak/kisak-mesa
apt update
apt upgrade
```

<br>

### AWSIM ends with coredump

If AWSIM ends with a coredump just by starting, there might be a GPU memory shortage.

Check with Nvidia-smi if the GPU memory usage is reaching its limit.

We recommend a GPU memory of more than 11GB.

<br>

## Other FAQ

You can also refer to the tier4:AWSIM troubleshooting.

[Refer here](https://github.com/tier4/AWSIM/blob/main/docs/DeveloperGuide/TroubleShooting/index.md)