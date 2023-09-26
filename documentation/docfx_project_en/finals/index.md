# [WIP] About the Finals

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