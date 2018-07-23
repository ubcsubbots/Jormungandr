# Line Detection

## Purpose
This package defines a ros node that takes in raw image data from a camera and determines whether there is a gate in the image, and if so what the orientation of the gate is.

The output of this node is a Message that contains the following information.

  float32 lateralDistanceFromFrontMarker    
  Lateral distance off the front most marker, positive is left negative is right   
  
  float32 distanceFromEndOfFrontMarker  
  Distance to end of front most marker, -1 if can't see end   
   
  float32 angleToParallelFrontMarker  
  Deviation angle from parallel of front most marker    
    
  float32 lateralDistanceFromRearMarker   
  Lateral distance off of rear marker, positive is left negative is right   
  
  float32 distanceFromEndRearMarker  
  Distance to end of rear marker, -1 if can't see end   
   
  float32 angleToParallelRearMarker  
  Deviation angle from parallel of rear marker   
    
## Method

The line detector scans the image viewed by the bottom camera. It breaks the image into the top half of the image and the bottom
half of the image. It looks for parallel lines that are reasonably far apart that could represent the bottom markers of the pool.
It then sorts these markers into the ones that are closest to the centre view of the camera. The decision node and worldstate node
can then use the forward marker and rear marker to line follow.
