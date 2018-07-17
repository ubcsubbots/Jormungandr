##Purpose

This node takes in raw image data from a camera and determines whether there is a gate in the image, and if so what the 
orientation of the gate is.

The output of this node is a Gate construct that consists of 3 boolean values and 6 float values that are described as follows:


          boolean detectedLeftPole:   false if left pole not seen, true if seen, 0 if not seen
          
          float angleLeftPole:      Angle from vertical centre of camera image to left pole

          float distanceLeftPole:   Distance to left pole, 0 if not seen

          boolean detectedRightPole:  false if right pole not seen, true if seen, 0 if not seen

          float angleRightPole:     Angle from vertical centre of camera image to right pole

          float distanceRightPole:  Distance to right pole, 0 if not seen

          boolean detectedTopPole:    0 if top pole not seen, 1 if seen

          float angleTopPole:       Angle from horizontal centre of image to top pole, 0 if not seen

          float distanceTopPole:    Distance to top pole, 0 if not seen


##Method

```

       TOP VIEW
       
       GATE       ANGLE TO POLE
                /        
   POLE O    | \/  O POLE
             |    /
             |   / 
             |  /  <--DISTANCE TO POLE
             | /
             |/          
          --------
          | ROBOT| 
          |      |
          --------
```


The gate detector used to process the image uses an interpolation function to estimate distance to the poles. Discrete data
points must be collected of the pixel width of the poles of the gate at various distances. You then must use this data to 
create an approximation function to map the pixel width of the poles to the distance from the pole.

The current interpolation function is a power function: 

(distance to pole) = interpolationConstant1 * (pixelWidthOfPole) ^ (interpolationConstant2)

This function is accurate between 3 and 15 metres in the simulator, it may need to be updated with real world data points.

The interpolation constants can be set in the roslaunch file.

There are 7 other parameters that can be set to tune the gateDetector node. 

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

