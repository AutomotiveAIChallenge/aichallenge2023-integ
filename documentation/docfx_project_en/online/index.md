# Online Environment
<br>

> [!REGISTER]
> Register from here!
> [https://www.jsae.or.jp/jaaic/en/index.php](https://www.jsae.or.jp/jaaic/en/index.php)

<br>

 &emsp;Scoring for this competition will be performed in an online environment equipped with a simulator and automatic scoring function.  
 &emsp;Please upload the packages you have created to the online environment by following the instructions below. After the upload is complete, the simulation will start in the online environment and the results will be presented.
## Upload Procedure to Online Environment
1. operation check

   &emsp; [LocalEnvironment](../local/index.html) page to check the operation.   
   &emsp;*Please check if the created package is organized in `aichallenge_submit`.
2. compression of source code

	 &emsp;Please compress `aichallenge_submit` by the following command, and `aichallenge_submit.tar.gz` will be generated in the docker directory.  
	```
	#In the aichallenge2023-sim directory
	cd docker
	bash create_submit_tar.sh
	````
	
3. upload to online environment    
	<img src="../../images/online/siteImage.png" width="100%">  
	 Go to [online environment](https://aichallenge.synesthesias.jp) and upload the `aichallenge_submit.tar.gz` created in step 3 from "Select File". Once the upload is complete, the source code build and simulation will be performed in order.
	* If the process is completed successfully, the grading will be displayed as completed and result.json will be available for downloading. Also, the distance points and times will be posted to the ranking again.
	* If the scenario execution finishes successfully but no score is output due to launch failure, etc., "No result" will be displayed, and if all checkpoints have not been passed, "Checkpoint not passed" will be displayed, and in either case, the score will not be used as the final time.
	* If the build fails, Build error is displayed. Please check the procedure again.
	* If the simulator fails to run, a Simulator error is displayed. In this case, there may be an internal error on the server side, so please upload the file again. If the error message is displayed repeatedly, please contact us.
	* The grading process will be performed 5 times per submission, and the result will be determined by the average of the 5 times.
	* You cannot upload new sources while the grading process is in progress. Uploading is limited to 3 times per day, and the number of times will be reset at midnight Japan time.
   
4. check the results  
	 After the evaluation is completed in the online environment, result.json will be available for downloading.
