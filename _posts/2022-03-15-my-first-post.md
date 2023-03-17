---
layout: default
title: ROS Toolbox for MATLAB
author: Hector Tovanche
categories: [Robotics, MATLAB, ROS]
tags: [MATLAB, ROS, T256]
---

# Using the ROS Toolbox for MATLAB with the T265 Intel camera

In this blog we will use the ROS toolbox for MATLAB to read a topic from a Jetson nano using the T265 Intel camera, and then we will subscribe to the /tf topic to get the position and orientation of the camera. Then we will plot the position of the camera in a 3D plot.

The ROS toolbox helps to integrate ROS in the MATLAB Environment. It provides a set of MATLAB functions and classes that enable you to interact with ROS nodes, topics, services, and parameters from MATLAB. The toolbox also provides a set of MATLAB functions that enable you to create and run ROS nodes from MATLAB. The toolbox is available for MATLAB R2018b and later releases.

The T265 camera from Intel is a small, lightweight camera, posses a high-accuracy tracking module, and is capable of tracking the camera’s position and orientation in 3D space. It is a perfect solution for robotics applications that require high-accuracy tracking, such as SLAM, visual-inertial odometry, and visual-inertial navigation.

Regarding to the structure of this project we will have the following:
1. Jetson nano with the T265 camera connected. Running the T265 camera node.
2. Computer with MATLAB running the ROS toolbox for MATLAB.
3. The computer with MATLAB will connect to the Jetson nano trough WiFi. 
4. The ROS toolbox for MATLAB will subscribe to the topic /tf from the Jetson nano and will plot the position of the camera in a 3D plot.

The following figure shows the structure of the project.

![Structure of the project](/assets/img/structure.png)

## Section 1

The first step is to install the ROS toolbox for MATLAB. To do this, we will use the following command:

```matlab
>> rostoolbox
```

This will install the toolbox and all the dependencies.
Also ROS toolbox for MATLAB is available in the MATLAB Add-On Explorer
 


After this, we will need to start the ROS master in the Jetson nano. But also is neccesary to Set the ROS_MASTER_URI and ROS_IP environment variables.
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

Convert quaternion to Euler angles

$$
\begin{aligned}
\mathrm{roll} &= \mathrm{atan2}\left(2(w x + y z), 1 - 2(x^2 + y^2)\right) \\
\mathrm{pitch} &= \mathrm{asin}\left(2(w y - z x)\right) \\
\mathrm{yaw} &= \mathrm{atan2}\left(2(w z + x y), 1 - 2(y^2 + z^2)\right)
\end{aligned}
$$
```matlab
>> q = [orientation.X orientation.Y orientation.Z orientation.W];
>> eul = quat2eul(q);
```



### Subsection 1.1

Phasellus vel mauris eget lorem blandit pharetra ut at velit.

### Subsection 1.2

Aliquam erat volutpat. Vestibulum ultrices gravida lectus, sit amet vestibulum odio facilisis vel.

## Section 2

Etiam non ex eget metus tincidunt suscipit. Sed interdum, sapien sed semper sagittis, eros risus elementum nisl, vitae convallis libero velit a velit.

...
