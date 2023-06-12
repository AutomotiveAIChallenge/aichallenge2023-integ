# AIChallenge2023 

## Introduction
&emsp;This site provides information for those who will participate in the preliminary round of the Automated Driving AI Challenge 2023 Integration Competition, including the competition rules and how to set up the environment for the competition.  
  
&emsp; Similar to the Automated Driving AI Challenge 2022 simulation competition that was held last year, this competition will use the Autoware.universe automated driving software and the automated driving simulator AWSIM. Please refer to the [Setup page](.../setup). /setup) and participate in the competition.
  
## About Competition
&emsp; You are required to follow the following steps to participate in the competition. 1.  
1. develop software to complete the given scenario  
2. validate the software created in step 1 in a local environment  
Upload the validated software to the online environment 4.  
4. simulation is performed online and time is measured   
    *The ranking will be determined based on the time of the simulation results of the last uploaded source code. (Invitation to the online environment will be provided at a later date)  
&emsp; For details of the competition rules, please refer to the [Rule page](. /rule) for details of the competition rules.

## About Autoware
&emsp; Autoware is an open source automated driving software using ROS2, which has a sensing function to acquire data from LiDAR and cameras, and a localization function to estimate the location of the vehicle by combining the sensing data as modules, The modules work in conjunction with each other to realize automated driving. This software is also being tested on public roads in Japan.  
&emsp; This competition will use Autoware.universe, which is a distribution for research and development within Autoware. For more information on other distributions and Autoware's development process to date, see [here](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware- concepts/difference-from-ai-and-auto/).
  ! [autoware](/images/intro/autoware.png)
  
## About AWSIM
 &emsp;AWSIM is an open source self-driving simulator that runs on Unity, ROS2 native, and supports Windows and Ubuntu, making it easy for anyone to simulate self-driving algorithms.
  When AWSIM is utilized in&emsp;Autoware, Autoware nodes subscribe to sensing data from AWSIM, and vehicle control information, which is the result of processing the received data in each module, is published to AWSIM to control vehicles on AWSIM. The vehicle on AWSIM is controlled.
 ! [awsim](/images/intro/awsim.png)
 
## Related Documentations.
 * [Official website of the Automated Ai Challenge](https://www.jsae.or.jp/jaaic/)
 * [Autoware.universe](https://github.com/autowarefoundation/autoware.universe)
 * [AWSIM](https://github.com/tier4/AWSIM)
 
## Page Links.
 * [Introduction](... /intro) /intro) About the Qualifying Competition.
 * [Setup](... /setup) Setup procedure
 * [Rule](... /rule) About the rules /rule) About the rules
 * [About Local Enviroment](... /local) /local) About Local Enviroment * [About Evaluation Enviroment](.../evaluation)
 * [About Evaluation Enviroment](.../evaluation) /evaluation) About Online Evaluation Enviroment
 * [Other](... /other) /other) Inquiries, how to share information among participants, etc.
 
 