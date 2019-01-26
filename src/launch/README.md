#Troubleshooting Robot

This guide is a step by step guide of troubleshooting the software side of the robot. At this point it includes the  
hsv_filter adjustment, gate_detection params, and the logic surrounding passing the gate.

First thing is first, when the master_launch is launched you should run:
rosrun rqt_graph rqt_graph  
to check that all of the nodes are talking to each other, it should look something like this.  

![alt text](https://raw.github.com/ubc-subbots/Jormungandr/src/model_update/robot_launch.png)  

Except there should be a camera node feeding into hsv_filter and the image_proc node running over the camera. If this is not  
the case you must remap the topics properly in the launch file, ie. rosrun rostopic list, check the topics that you want,  
and change the remap tags in the robot_full.launch file.  

Next is adjusting the hsv_filter. To do this the robot must be in the water and able to see something like the gate. You  
must launch the camera node, image_proc node, and the hsv filter node. You then must run:
rosrun rqt_reconfigure rqt_reconfigure  
and rosrun rviz rviz  
In rviz go to add topic and select the hsv_filter/output topic. In rqt_reconfigure click the hsv_filter node and adjust the
HSV parameters until you can clearly see the target. Try to reduce the noise in the image as much as possible while still  
seeing the target clearly.  

With this done you should be able to launch the robot, if the robot spins continuously without detecting the gate then  
the gate parameters must be adjusted.  This will be done in the roslaunch file. Here is a description of the gate_detect  
parametrs:

cannyLow: Can be set in the ros launch file and updated using ros dynamic reconfigure. Relates to the low threshold of the 
          Canny Edge detector. Raising this value will decrease the amount of lines detected in the image by requiring a 
          higher differential in pixel intensity to be considered an edge.

poleMax: Can be set in the ros launch file and updated using ros dynamic reconfigure. This number indicates the maximum pixel 
          count that two lines that are detected in the image can be to be considered a pole of the gate. Decreasing this 
          value will decrease the amount of poles detected.

houghLinesThreshold: Can be set in the ros launch file and updated using ros dynamic reconfigure. This number sets the 
                      maximum required line intersections required for a line to be detected in the opencv hough line
                      detector function. Increasing this number will decrease the amount of lines detected.

houghLinesMinLength: Can be set in the ros launch file and updated using ros dynamic reconfigure. This number indicates the
                      minimum length a line can be to be considered a line by the opencv hough line detector.

houghLinesMaxLineGap: Can be set in the ros launch file and updated using ros dynamic reconfigure. This number indicates the
                      maximum gap that can occur between 2 lines to be considered 1 line by the open cv hough line detector.

lowVertThresh: This number cannot be set in a ros launch file or dynamically reconfigured. It represents the maximum
               deviation from perfectly vertical that a line can have to be considered a vertical line. Increasing this 
               number will increase the amount of lines detected.

lowHorThresh: This number cannot be set in a ros launch file or dynamically reconfigured. It represents the maximum
               deviation from perfectly horizontal that a line can have to be considered a horizontal line. Increasing 
               this number will increase the amount of lines detected.
               
The first steps should be to increase the lowVertThresh, and the lowHorThresh and see if the robot starts to looks like  
it's detecting something. Next decrease the houghLinesThreshold and houghLinesMinLength, and the houghLinesMaxLineGap.  
These may need to be fiddled  with to get the robot to recognize the gate.  

If the robot is seeing the gate, but not passing through it properly, then you must adjust the parameters regarding  
passing the gate. This will be done in the src/decision/constants/constants.yaml file. You must adjust the target values  
and error tolerance values to mimic what the robot is failing on.