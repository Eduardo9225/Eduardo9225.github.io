---
layout: default
title: ROS Toolbox for MATLAB
author: Hector Tovanche
categories: [Robotics, MATLAB, ROS]
tags: [MATLAB, ROS, T256]
---

# Introduction

In this blog we will use the ROS toolbox for MATLAB to read a topic from a Jetson nano using the T265 Intel camera, and then we will subscribe to the /tf topic to get the position and orientation of the camera. Then we will plot the position of the camera in a 3D plot.

## Section 1

The first step is to install the ROS toolbox for MATLAB. To do this, we will use the following command:

```matlab
>> rostoolbox
```

This will install the toolbox and all the dependencies. After this, we will need to start the ROS master in the Jetson nano. But also is neccesary to Set the ROS_MASTER_URI and ROS_IP environment variables.
to do this we will use the following commands:

```bash
$ export ROS_MASTER_URI=http://localhost:11311
$ export ROS_IP=IP_ADDRESS
```
Where IP_ADDRESS is the IP address of the computer where we are running MATLAB. and the ROS_MASTER_URI is the IP address of the Jetson nano.

Then we will need to start the ROS master in the Jetson nano.

```bash
$ roscore
```

Then we will to launch the T265 camera in the Jetson nano. To do this, we will use the following command:

```bash
$ roslaunch realsense2_camera rs_t265.launch
```

We can check if the camera is working by using the following command:

```bash
$ rostopic echo /camera/odom/sample
```

Also we can check the position and orientation of the camera by using the following command:

```bash
$ rostopic echo /tf
```

Another way to chech and visualize the position and orientation of the camera is by using rviz with the following command:
    
    ```bash
    $ rosrun rviz rviz
    ```
        
        Then we will need to add the topic /tf to the rviz window.


If Rviz displays the camera position and orientation, then we can continue with the next step.

In Matlab is necessary to add the ROS_IP and ROS_MASTER_URI to the MATLAB environment variables. To do this, we will use the following commands:

```matlab
>> setenv('ROS_IP', 'IP_ADDRESS')
>> setenv('ROS_MASTER_URI', 'http://localhost:11311')
```

Where IP_ADDRESS is the IP address of the computer where we are running MATLAB. and the ROS_MASTER_URI is the IP address of the Jetson nano. Later we can add this commands to the startup.m file to run it automatically when we start MATLAB or we can create a script and add this commands to the begining.

Then we will need to start the ROS node in MATLAB. To do this, we will use the following command:

```matlab
>> rosinit
```
We can check if the ROS node is running by using the following command:

```matlab
>> rostopic list
```

If displays the list of topics, then we can continue with the next step.

Now we will need to read the topic from the Jetson nano. To do this, we will use the following command:

```matlab
>> odom = rossubscriber('/odom/sample');
```

Then we will need to read the data from the topic. To do this, we will use the following command:

```matlab
>> odomdata = receive(odom);
```

Now we will need to read the position and orientation of the camera. To do this, we will use the following command:

```matlab
>> position = odomdata.Pose.Pose.Position;
>> orientation = odomdata.Pose.Pose.Orientation;
```

### Subsection 1.1

Phasellus vel mauris eget lorem blandit pharetra ut at velit.

### Subsection 1.2

Aliquam erat volutpat. Vestibulum ultrices gravida lectus, sit amet vestibulum odio facilisis vel.

## Section 2

Etiam non ex eget metus tincidunt suscipit. Sed interdum, sapien sed semper sagittis, eros risus elementum nisl, vitae convallis libero velit a velit.

...
