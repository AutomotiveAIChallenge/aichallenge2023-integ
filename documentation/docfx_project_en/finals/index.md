# About the Finals

<br>

## Introduction
In the final competition, participants will operate a golf cart-style autonomous vehicle using their development laptops to showcase the performance of their self-driving software. All operations, from connecting the development laptop to the vehicle to initiating autonomous driving, are to be performed by the participants.
<br>

> [!Video https://www.youtube.com/embed/8oF-XcbsJes?si=mD5LQWd28DEj5L7L]

<br>

<div align="center">
  <img src="../../images/operation/aichal-2022-expl.png" alt="Last year's run" width="500">

  _Driving situation_<br>
  （refer: https://www.jsae.or.jp/jaaic2022/final.html "Explanation of　the competition"）
</div>

<br>

## About the competition vehicle

<br>

<div align="center">
  <img src="../../images/operation/aichal-vehicle.png" alt="Competition Vehicles" width="500">

  _Vehicles used for competition_
</div>

<br>

- The vehicle used in the competition is fitted with the same sensors (LiDAR) as the one used in the E2E space during the qualifying round. However, the vehicle’s kinematic properties differ from the model in the simulation.
- The vehicle comes with a network interface linked to the LiDAR and a CAN interface. Autonomous driving is facilitated by connecting a development laptop to these interfaces.
- A mentor and a safety driver will accompany the competition vehicle. The mentor will handle the interface connections and aid in the operation of the autonomous driving software, while the safety driver will manually maneuver the vehicle and oversee it during autonomous driving.
- The vehicle offers two control modes: `automatic` and `manual`. The safety driver has the ability to switch between these modes.
- During autonomous driving, the safety driver has the capability to transition the vehicle's control mode from `automatic` to `manual` by pressing the brakes. This action is termed an override. The safety driver will initiate the override if continuing autonomous driving is deemed unsafe.

## Source Code Build Method for Final Competition PC

On the final competition PC, it is assumed that the source code will be built locally, not in docker. For installing dependencies and building the source code, please execute the following commands:

```bash
cd /home/autoware/aichallenge2023-integration-final

rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## For Final Tournament Participants (Updated on 2023/09/28)

Please make sure all the participants of the final tournament thoroughly check the following before the day of the tournament.

### Support System for Participating Teams

Each team will receive support from TIERIV and MAPIV in the form of two staff riding with: a Safety Driver (SD), a Safety Operator (SO). Your SO will also help your setup.

- Role of SD (Safety Driver)
The role of the SD is to manually drive the golf cart and monitor the vehicle during autonomous driving.  
**An experienced staff member will supervise autonomous driving as SD. If an override is performed, please respect the judgment of your SD.**

- Role of Mentor and SO (Safety Operator)
While the SO acts as a mentor to participants, they also monitor autonomous driving.  
**If your SO determines that continuing autonomous driving is dangerous, they have the responsibility to operate the emergency stop button. If the emergency stop button is pressed, please respect the judgment of your SO.**

### Request for Preliminary Simulation

**For the safe operation of the competition, we request you to execute the Planning Simulation before coming to the test field.**  
*How to execute Planning Simulation*  

1. Open a terminal window and run the following commands:
    For those who have the distributed PC, please execute the following command. For those who do not have the distributed PC and wish to practice, please replace the vehicle model with a distributable model by [clicking here](https://github.com/AutomotiveAIChallenge/aichallenge2023-integration-final/blob/178f9a05d77560f51df4bde915d15c6300f1d99a/scripts/vars/vehicle.env#L1) and replacing `export VEHICLE_MODEL=golfcart`.
    ```bash
    source /home/autoware/aichallenge2023-integration-final/install/setup.bash
    cd /home/autoware/aichallenge2023-integration-final/scripts
    ./launch_psim.sh

    ```

2. Use `2D Pose Estimation` to position the vehicle at the starting point.
3. Open a new terminal window and run the script `~/aichallenge2023-integration-final/scripts/set_obstacles.sh`. Then confirm that virtual obstacles are placed in the area of task 3-1 (slalom). *If you cannot find the script, please merge [this commit](https://github.com/AutomotiveAIChallenge/aichallenge2023-integration-final/commit/49ec129db8f26485fffed7daef85c5da1649998b) into your repository.
4. Set the goal and start autonomous driving.
5. Confirm that the vehicle does not exhibit dangerous behavior (exceeding speed limits, ignoring obstacles).

For more details about Planning Simulation, please refer [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/)

### Criteria Regarding Weather Conditions

- If the wind speed is higher than 5m/s, Task 2 involving smoke will be canceled.
- If 11/12 is rainy, the competition will be rescheduled to 11/19.
- According to the weather condition, pre-practice can be canceld and no alternative day will be given. We appreciate your understanding.

### Notice Regarding Fairness of the Competition

- About vehicle characteristics  
The vehicles provided have already been tuned. Sensor positions and steering offsets parameters have been tuned at TIERIV. However, you may add adjustments at your responsibility.
- Handling the task 2, smoke  
This tournament is themed around factory transportation, aiming for stable driving under smoke. We are trying to recreate the environment in outdoor, so there may be environmental differences depending on the timing of the drive. We appreciate your understanding.

### Additional Feature and Updated Map

- An additional function has been added so that automatically interrupts (Disengage) the driving in case the vehicle deviates from the course.
- New map is available. Please be prepared to appropriately replace the maps as needed. You may download new map from the following Slack link.  
[Download New Map](https://aichallenge2023-integ.slack.com/files/U05CFHNAZ8B/F05U4K4QMBJ/lanelet2_map.osm))
