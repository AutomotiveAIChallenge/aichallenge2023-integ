# [WIP]Operation (Finals Competition)

**Note: This page is a work in progress. The information on this page is provisional and may change in the future.**

This page describes the procedure for running the competition vehicle (an automated golf cart, hereafter referred to as “golf cart”) on a laptop with Autoware installed.

※The explanation is based on the assumption that the repository for development is located in `/home/autoware/aichallenge2023-integration-final`.


## Steps to Start Automated Operation

This section describes the flow of the competition participants from connecting the development laptop to the vehicle until the start of automated driving.

**※Please be sure to read [the safety precautions](#安全に関する注意点) before starting the vehicle.**

1. (Competition participants) Open a terminal and execute the following command to start Autoware.
    ```
    $ cd ~/aichallenge2023-integration-final/scripts
    $ ./run_autoware_on_vehicle.sh
    ```

2. (Competition participant) Start self-position estimation.
   1. Input 2D pose estimate from rViz.
   2. Confirm that the `Localization` display on the `AutowareStatePanel` is `INITIALIZED`.
   <img src="../images/operation/loc_initialized.png" alt="Localization INITIALIZED State" width="300">

3. Specify the (Competiton participant) goal point.
   1. Open a terminal and execute `~/aichallenge2023-integration-final/scripts/set_goal.sh` to set the goal point.
   2. Confirm with rViz that the route to the goal is drawn.

4. (Safety driver)Set the golf cart mode to `automatic mode`.

5. (Competiton participant) Enable vehicle control by Autoware.
   1. Click on the `Enable` button of the `AutowareControl` in the `AutowareStatePanel` after receiving the mentor’s instructions.
   2. Confirm that the display of `AutowareControl` in the `AutowareStatePanel` is `Enabled`.
   <img src="../images/operation/autoware_control_enabled.png" alt="Autoware Control Enabled" width="300">

6. (Safety Driver) Checks the steer behavior, confirms that the golf card is successfully accepting Autoware control, and communicates Autoware’s autopilot mode permission to the participant.

7. (Competiton participant) Sets Autoware to automatic operation mode.
   1. Speaks to the safety driver, “I will start automatic operation”.
   2. Click the `AUTO` button in the `OperationMode` of the `AutowareStatePanel`.
   3. Confirm that the `OperationMode` display on the `AutowareStatePanel` is set to `AUTONOMOUS`.
   <img src="../images/operation/operation_mode_auto.png" alt="Operation Mode Auto" width="300">

### When an override (when the driver stops the vehicle) occurs

If it is anticipated that the vehicle will run into a curb or come in contact with an obstacle during automatic operation, the safety driver will apply the brakes and switch the vehicle control from automatic operation to manual operation. This is called an override. 
If the safety driver overrides during a segment of Task 1 or Task 2, the following procedure is used to start the next task.
1. (Safety Driver) Notify the competitors that they have overridden.
2. (Safety Driver) Sets the golf cart mode to `manual mode`.
3. (Safety Driver) Moves the vehicle to the starting point of the next task.
4. (Safety Driver) Moves the vehicle to the starting point of the next assignment.
5. Confirm that the display of `AutowareControl` in the `AutowareStatePanel` is `Disable`.
6. Resume an [automated operation from step](#steps-to-start-automated-operation) No4(**(Safety Driver)Set the golf cart mode to `automatic mode`.**)

## Safety precautions

- Failure to switch to automatic operation mode may occur due to a poor HW connection or other reasons. Failure to switch to the automatic operation mode is determined by the safety driver. If this event occurs, follow the mentor’s instructions and redo the procedures up to the start of automatic operation.
- Participants who are not riding in the vehicle should stay on the sidewalk during the automated driving. No one is allowed on the route or around the vehicle.
- Please be sure to hold onto the upper handrail during the automated driving.
- Please secure your laptop computer used for automatic driving with a band. If you have difficulty typing on the keyboard, use the USB wireless keyboard provided for loan.