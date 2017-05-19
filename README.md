# tensor_field_nav (Tensor Field Based Navigation)

## Introduction

Tensor_field_nav package is developed for autonomous mapping of unknown indoor scenes by a mobile robot holding an RGBD camera. The key idea is to utilize 2D directional fields to guide robot movement. We compute and update a geometry-aware tensor field constrained by the currently reconstructed scene. The 3D scene geometry (i.e., the known surfaces) is projected to the floor plane. A set of 2D tangential constraints along the projected boundaries is extracted and used to compute/update the tensor field. The robot path is formed by particle advection over the tensor field, which is inherently obstacle avoiding.

During online scanning, the tensor field is updated in real-time, conforming to the incrementally reconstructed scene. To ensure a smooth robot path when advecting over the time-varying field, we propose a space-time optimization of tensor fields via imposing both spatial smoothness and temporal coherence. There are several important advantages of tensor field guided navigation. First, tensor fields are orientation-free and thus contain much less singularities (degenerate points), as compared to vector fields which are predominantly used in the literature. Fewer singularities lead not only to smoother path advection, which is critical for quality reconstruction, but also to more efficient navigation due to less ambiguity. In addition, tensor fields are sink-free, avoiding the issue of local trapping. Most importantly, the topological skeleton of a tensor field, comprised of all degenerate points and the separatrices connecting them, can be viewed as a routing graph. With this global structure, one can achieve global path planning for efficient scene scanning.

## Requirements

This package depends on CUDA and OpenCV. Please install these two libraries first. CUDA version 7.5 and OpenCV 2.4.8 version were tested and thus recommended.


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




