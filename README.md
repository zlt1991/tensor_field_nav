# tensor_field_nav (Tensor Field Based Navigation)

## Introduction

Tensor_field_nav package is used for autonomous mapping of unkonown indoor scenes guided by time-varing tensor field. It leverages rgbd sensor like kinect to scan environment, and receives scene geometry information from octomap. Based on the partially mapping scene, a 2D tensor field is computed on the floor plane, under the constraints of 2d grid map. The robot is guided by the field with smooth paths, which are locally formed with advection and globally planned with help of the field topology.

## Requirements

This package relies on some third dependencies, it requires installation of CUDA, Opencv.


## Download and Installation

Dowload tensor_field_nav package with the following command to your catkin workspace.
```
catkin_ws/src$ git clone 
catkin_ws/src$ cd ..
catkin_ws$ catkin_make
```
## Run
roslaunch tensor_field_nav_core complete_run.launch

roslaunch octomap_tensor_field octomap_mapping.launch




