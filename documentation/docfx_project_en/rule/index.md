# Rule
This competition is an online simulator to compete the score calculated from the driving result of the automatic vehicle.

## Driving Vehicle
A golf cart will be used as the driving vehicle.
  <img src=". /images/rule/vehicle.png" width="70%">

## Scenario
  Here is the course that will challenge you.
  <img src=". /images/rule/kadai.png" width="70%">
### Challenge 1: Stop when an obstacle is detected (cardboard)
Condition: Stop before the obstacle  
Penalty: X% discount from the distance point if you fail to stop
  <img src=". /images/rule/kadai1.png" width="70%">
### Assignment 2: Driving in the presence of smoke
Condition: Do not stop due to smoke    
Penalty: Y% discount from the distance point if stopped
  <img src=". /images/rule/kadai2.png" width="70%">
### Challenge 3: Driving on a narrow road (S-curve, L-crank)
Condition: Complete the course without colliding with any obstacle and without deviating from the designated route.  
Penalty: If the vehicle collides with an obstacle or strays from the designated route, the race is over.
  <img src=". /images/rule/kadai3.png" width="70%">
  
## Detail of Rule  
### Rank-determination Rules
* Ranks are determined according to distance points.  
*Distance points = overall distance traveled from the starting point (m) * (1.0 - Issue 1 penalty - Issue 2 penalty)*
* If the distance points are the same, the competitor with the shortest running time in the task 3 area will be ranked higher (if he/she runs to the end of the task).

### Limitation
* Speed limit is 0~5km/h
* If you exceed the speed limit by more than 2km/h, you will be disqualified = [points_map_hakodate](... /...) /... /... /points_map_hakodate.pcd) 0 points
* Placement may be slightly random.

### Confirmation of time
````
# In the Rocker container
source /aichallenge/aichallenge_ws/install/setup.bash
ros2 topic echo /score/result
# The following is displayed when a goal is scored or disqualified.
# score: 164699
# has_finished: true
# has_collided: false
# has_park_failed: false
# check_point_count: 2
# the value of `score` is the final time.
# if `has_finished` is false, you are disqualified.
```