## Evaluation Enviroment

## Outline of Execution Flow in an Online Environment
　To calculate the score, please submit only the package aichallenge_submit from the web page of the online evaluation environment, and the score will be automatically scored. After submission, the online evaluation environment uses the evaluation/ script below to evaluate the package in the following steps. 1.
1. deployment of aichallenge_submit  
　The uploaded aichallenge_submit.tar.gz file will be placed under evaluation/.
  
2. docker build  
　evaluation/build.sh will be executed to create the docker image defined in the evaluation/Dockerfile. The steps to create this image are as follows. 1.
    1. extract the submitted aichallenge_submit.tar.gz to /aichallenge/aichallenge_ws/src/aichallenge_submit  
    2. run rosdep install and colcon build    
          
3. run simulation  
　The simulator is launched in the online evaluation environment and the simulation is started.  
　In the container, the following is performed by executing evaluation/main.bash: 1.
    1. start the ROS2 nodes
    2. start of scenario
    When executed in evaluation/run.sh, the result (score.json) is saved under evaluation/output.
    

## Procedures for Submitting Source Code
1. compress source code
　Compress the source code in aichallenge_submit.
　
   ````
   cd evaluation
   sh create_submit_tar.sh
   ````
   Make sure that the compressed file is created in evaluation/aichallenge_submit.tar.gz. 2.
     
2. make sure that the file can be automatically executed in docker at ``evaluation/
　Before uploading to the online evaluation environment, please confirm that you can build and run in a Docker container as in the online environment using your local environment by following the steps below.  
    First, make sure the following files are located under evaluation/.
    * aichallenge_submit.tar.gz    
Next, build the docker image containing the aichallenge_submit you created.
   ````
   sh build.sh
   ```

      After the build is complete, use run.sh to launch the docker container and execute the scoring flow.
   
   ```
   sh run.sh
   ```
   Finally, check the score output to evaluation/output/score.json. 3.
Upload the source code from the online evaluation environment web page
After logging in to the [web page](), follow the instructions on the screen and upload the aichallenge_submit.tar.gz file created in (1).  
  
    After the upload is completed, the source build and simulation runs will be performed in sequence. 
    * If the simulation finishes successfully, the message "Scoring complete" will be displayed, and the times for each of the distribution and evaluation scenarios will be displayed. The time of the last uploaded evaluation scenario is used as the final time in the ranking.
    * If the scenario execution is successfully completed, but no score is output due to launch failure, etc., "No result" will be displayed, and if all checkpoints have not been passed, "Checkpoint not passed" will be displayed.
    * If the build fails, Build error is displayed. Please reconfirm that you can build the Docker image by following the steps (1) and (2).
    * If the simulator fails to run, a Simulator error will be displayed. In this case, there may be an internal error on the server side, so please upload the image again. If the error message is displayed repeatedly, please contact us.
    /other) Inquiries, how to share information among participants, etc.
     You cannot upload new sources while the grading process is in progress. Uploading is limited to 3 times a day and will be reset at midnight Japan time.