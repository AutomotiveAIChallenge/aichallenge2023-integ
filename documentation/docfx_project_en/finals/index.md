# [WIP] About the Finals

<br>

## Introduction
In the final competition, participants will drive a golf cart-type self-driving car using their own development laptop computers to compete in the performance of the self-driving software they have developed. Participants are required to perform all operations from connecting the development laptop to the vehicle to the start of automatic driving.

<br>

> [!Video https://www.youtube.com/embed/8oF-XcbsJes?si=mD5LQWd28DEj5L7L]

<br>

<div align="center">
  <img src="../images/operation/aichal-2022-expl.png" alt="Last year's run" width="500">

  _Driving situation_<br>
  （refer: https://www.jsae.or.jp/jaaic2022/final.html "競技解説 Explanation of　the competition"）
</div>

<br>

## About the competition vehicle

<br>

<div align="center">
  <img src="../images/operation/aichal-vehicle.png" alt="Competition Vehicles" width="500">

  _Vehicles used for competition_
</div>

<br>

- The vehicle used in the competition is equipped with the same sensors (LiDAR) as the vehicle operated in the E2E space during the qualifying round. On the other hand, the vehicle’s kinematic characteristics are not identical to the model in the simulation.
- The vehicle is equipped with a network interface and a CAN interface connected to the LiDAR. Automatic driving is performed by connecting a development laptop to these interfaces.
- The vehicle used for the competition will be accompanied by a mentor and a safety driver. The mentor will connect the interfaces and assist in operating the automated driving software, while the safety driver will move the vehicle manually and monitor the vehicle during automated driving.
- The vehicle can be controlled in two modes, `automatic` and `manual`, which can be switched by the safety driver.
- During automatic driving, the safety driver can switch the vehicle control mode from `automatic` to `manual` by applying the brakes. This is called an override. The safety driver performs the override when it is deemed dangerous to continue the automatic driving.