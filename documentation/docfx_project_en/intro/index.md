## AI Challenge 2023 Integration Competition Preliminary Round

<br>

<!-- > [!REGISTER]
> Register from here!
> [https://www.jsae.or.jp/jaaic/en/index.php](https://www.jsae.or.jp/jaaic/en/index.php)

<br> -->

## Introduction
As in the Automated AI Challenge 2022 simulation competition held last year, the qualifying competition for the Automated AI Challenge 2023 Integration Competition will use the Autoware.universe automated driving software and the AWSIM automated driving simulator. The [Setup page](../setup) and participate in the competition.
  
## About Competition
&emsp; You are required to follow the following steps to participate in the competition.
1. develop software based on Autoware.universe to complete the given scenario  
2. validate the software created in step 1 in the local environment  
3. Upload the validated software to the online environment
4. simulation is performed online and times are measured   
    *The ranking will be determined based on the simulation result time of the last uploaded source code.
    (Invitation to the online environment will be provided at a later date)  


## About Autoware
&emsp; Autoware is an open-source self-driving software that uses ROS2, a sensing function that acquires data from LiDAR and cameras, and a localization function that estimates the location of the vehicle by combining the sensing data as modules, The modules work in conjunction with each other to realize automated driving. This software has been tested on public roads in Japan.  
&emsp; This competition will use Autoware.universe, which is a distribution for research and development within Autoware. For more information on other distributions and Autoware's development process to date, see [here](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/difference-from-ai-and-auto/).
  
## About AWSIM
 &emsp;AWSIM is an open source self-driving simulator that runs on Unity, ROS2 native, and supports Windows and Ubuntu, making it easy for anyone to simulate self-driving algorithms.
  When utilizing AWSIM in&emsp;Autoware, Autoware nodes subscribe to sensing data from AWSIM, process the received data in each module, and publish the results (vehicle control information) to AWSIM to The vehicle is controlled on AWSIM. For more information, please check [here](https://github.com/tier4/AWSIM).
 ![awsim](../../images/intro/awsim.png)
 
## Related Documentations.
 * [Official website of the Automated Ai Challenge](https://www.jsae.or.jp/jaaic/)
 * [Autoware.universe](https://github.com/autowarefoundation/autoware.universe)
 * [AWSIM](https://github.com/tier4/AWSIM)
 
## Page Links.
 * [Introduction](../intro) About the Qualifying Competition
 * [Setup](../setup) Setup procedure
 * [Rule](../rule) About the rules of the tournament
 * [LocalEnvrionment](../local) About local environment
 * [OnlineEnvrionment](../online) About online environment
 * [Other](../other) How to contact us, etc.
 