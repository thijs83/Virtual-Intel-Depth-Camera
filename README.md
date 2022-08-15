= Virtual-Intel-Depth-Camera
:toc:
:toc-placement!:

toc::[]

# Introduction

This example shows the implementation of a Intel Realsense D455 camera inside a simple virtual world made in Gazebo. The camera publishes it's pointcloud and image to ROS. One C++ example node detects objects in the pointcloud and publishes these as markers. Another Python example node detects the orange cones in the images and marks these in the image that is published by the node. This image, together with the pointcloud and the marker detections is visualized in Rviz. 

Gazebo             |  Rviz
:-------------------------:|:-------------------------:
![](.. image:: screenshot_gazebo.png)  |  ![](.. image:: screenshot_rviz.png)




The files included in this example are:

```
$ tree
.
    .src
    .src

# Requirements

This example is made using ROS Noetic and there is no garuantee that it works with other distributions. Both OpenCV and PCL libraries are needed for the detection nodes.  



# Install

To install the packages follow:
[source,bash]
----
$ mkdir catkin_ws
$ cd catkin_ws
$ git clone https://github.com/thijs83/Virtual-Intel-Depth-Camera.git
$ catkin_make
----


# Get started


# Packages

## realsense_d455

This package contains all models to setup the virtual environment and publishing the camera output to ROS

## pointcloud_filter

This package contains an example pointcloud filter in c++ that detects objects in the pointcloud retrieved from the camera.

## image_filter 