# tensor_field_nav (Tensor Field Based Navigation)

## Introduction

Tensor_field_nav package is used for autonomous mapping of unkonown indoor scenes guided by time-varing tensor field. It leverages rgbd sensor like kinect to scan environment, and receives scene geometry information from octomap. Based on the partially mapping scene, a 2D tensor field is computed on the floor plane, under the constraints of 2d grid map. The robot is guided by the field with smooth paths, which are locally formed with advection and globally planned with help of the field topology.

## Requirements

This package relies on some third dependencies, it requires installation of CUDA, Opencv.


## Download and Installation

Dowload tensor_field_nav package with the following command to your catkin workspace.

git clone http://github.com/zlt1991/tensor_field_nav.git


you should first check the executive authority of some files:

sudo chmod +x octomap_server/cfg/OctomapServer.cfg

sudo chmod +x tensor_field_nav_core/listen.py


you can then compile it with:

catkin_make -DCATKIN_WHITELIST_PACKAGES="pure_pursuit_controller"

catkin_make -DCATKIN_WHITELIST_PACKAGES=""


