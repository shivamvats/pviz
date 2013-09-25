pviz
====

PR2 Visualization Library

by Benjamin Cohen (with new features added by Ellis Ratner)

A very easy to use library that allows you to visualize the PR2 robot in
Rviz using visualization markers - with one line of code. It also has easy
to use functions to visualize all geometric shapes, meshes and text. This
library is great for debugging. 

PViz relies on the KDL for kinematics which means that TF and Gazebo are not
needed.


#### Visualization Functions:

  * visualizeRobot
  * visualizeRobotWithTitle
  * visualizeGripper
  * visualizeTrajectory

  * visualizePose
  * visualizeSphere
  * visualizeCube
  * visualizeLine
  * visualizeMesh

#### Marker Functions:

  * getRobotMarkerMsg
  * getRobotMeshesMarkerMsg
  * getGripperMeshesMarkerMsg
  * getCubeMsg

  * publishMarker
  * publishMarkerArray
  * deleteVisualizations

(Many of the functions above are overloaded with many different options of parameters to use)



