# Online Environment
<br>

> [!REGISTER]
> Register from here!
> [https://www.jsae.or.jp/jaaic/en/index.php](https://www.jsae.or.jp/jaaic/en/index.php)

<br>

 &emsp; In this competition, scoring will be conducted in an online environment equipped with simulators and automatic grading features.   
 &emsp; Follow the steps below to upload the package you have created to the online environment. After the upload is complete, the simulation will start in the online environment, and the results will be presented.
 
## Upload Procedure to Online Environment
1. Functionality Check

   &emsp; Please perform a functionality check simulating the behavior in the online environment, using only aichallenge_submit for the upload.
   	1.  Preparations (Compression of aichallenge_submit and generation of the output folder for results)
   	```
	# In the aichallenge2023-sim directory
	cd docker/evaluation
	bash advance_preparation.sh
	```
	2. Build the Docker image
	```
	# In the aichallenge2023-sim/docker/evaluation directory
	bash build.sh
	```	
	3. Launch AWSIM
	4. Start the Docker container (After launch, autoware will start automatically, and autonomous driving will begin)
	```
	#In the aichallenge2023-sim/docker/evaluation directory
	bash run_container.sh
	```	
	5. Verify result.json  
		Once the evaluation is complete, result.json will be stored in the `output` folder.
		
2.  Upload to the Online Environment
	<img src="../images/online/siteImage.png" width="100%">  
	 &emsp; Access the [online environment](https://aichallenge.synesthesias.jp) and upload the aichallenge_submit.tar.gz file you created in step 1 by clicking "Choose File." Once the upload is complete, the source code will be built, and the simulation will be run in that order.
	
	* If the process finishes successfully, "Scoring complete" will be displayed, and you can download the result.json file. The distance points and time will be displayed on the ranking.
	* If the scenario execution finishes successfully, but there is no score output due to a failed launch or other issues, it will be marked as "No result" and will not be used as the final time.
	* If the build fails, "Build error" will be displayed. Please review the instructions and try again.
	* If the simulator execution fails, "Simulator error" will be displayed. In this case, there may be an internal error on the server, so please try uploading again. If the issue persists, please contact support.
	* The scoring process will be run three times for each submission, and the highest score will be used as the final result. The ranking will be based on the highest score obtained in the scoring process so far.
	* You cannot upload new sources while the scoring process is ongoing.
	* You can upload up to three times per day, and the count will reset at 0:00 Japan time.
   
4.  Check the Results  
	&emsp; After the evaluation is completed in the online environment, you can download the result.json file and review the results.