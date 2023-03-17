---
layout: default
title: ROS Toolbox for MATLAB
author: Hector Tovanche
categories: [Robotics, MATLAB, ROS]
tags: [MATLAB, ROS, T256]
---
<script
  src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"
  type="text/javascript">
</script>
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

Matlab posses a function to convert quaternion to Euler angles. To do this, we will use the following command:
```matlab
>> q = [orientation.X orientation.Y orientation.Z orientation.W];
>> eul = quat2eul(q);
```

Also we can implement our own function to convert quaternion to Euler angles. To do this, we will use the following code:

```matlab
function [roll, pitch, yaw] = quat2eul(q)
% Convert quaternion to Euler angles
% Input:
%   q = 4-element quaternion vector in the order [w x y z]
% Output:
%   roll = rotation around x-axis in radians
%   pitch = rotation around y-axis in radians
%   yaw = rotation around z-axis in radians

% Extract components of quaternion
w = q(1);
x = q(2);
y = q(3);
z = q(4);

% Compute roll, pitch, and yaw
roll = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));
pitch = asin(2*(w*y - z*x));
yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2));
end
```
This function accepts a 4-element quaternion vector in the order [w x y z] and returns the rotation around x-axis, y-axis and z-axis in radians.
We can test this function by using the following command:

```matlab
>> [roll, pitch, yaw] = quat2eul(q);
```

Then we can print the results by using the following command:

```matlab
% Print the Euler angles in degrees
fprintf('Roll: %f degrees\n', roll*180/pi);
fprintf('Pitch: %f degrees\n', pitch*180/pi);
fprintf('Yaw: %f degrees\n', yaw*180/pi);
```

Now we can focus in the plot part of the project.
We can simulate the RPY angles to see how the plot will look like. To do this, we will use the following code:

```matlab
% Simulate RPY angles
roll = 30; % in degrees
pitch = 20; % in degrees
yaw = 10; % in degrees

% Convert to radians
roll = roll*pi/180;
pitch = pitch*pi/180;
yaw = yaw*pi/180;
```

Now we will define the length of the axes. To do this, we will use the following code:

```matlab
% Define the length of the axes
arrow_length = 1;
```

The next step is to define the reference frame axes.

```matlab
% Define the reference frame axes
x_axis = [arrow_length;0;0];
y_axis = [0;arrow_length;0];
z_axis = [0;0;arrow_length];
```
Now we can define the body frame axes. To do this, we will use the following code:

```matlab
% Define the body frame axes
x_body = [1;0;0];
y_body = [0;1;0];
z_body = [0;0;1];
```

Then is neccesary to define the rotation matrix. To do this, we will use the following code:

```matlab
% Rotate the body axes by the roll, pitch, and yaw angles
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
R = Rz*Ry*Rx;
x_body = R*x_body;
y_body = R*y_body;
z_body = R*z_body;
```


Now we can plot the reference frame axes. To do this, we will use the following code:

```matlab
% Plot the reference frame axes
quiver3(0,0,0,x_axis(1),x_axis(2),x_axis(3),'r','LineWidth',2,'MaxHeadSize',0.5);
hold on;
quiver3(0,0,0,y_axis(1),y_axis(2),y_axis(3),'g','LineWidth',2,'MaxHeadSize',0.5);
quiver3(0,0,0,z_axis(1),z_axis(2),z_axis(3),'b','LineWidth',2,'MaxHeadSize',0.5);
```

Then we can plot the body frame axes. To do this, we will use the following code:

```matlab
% Plot the body frame axes

% Plot the body axes
quiver3(1,5,2,x_body(1),x_body(2),x_body(3),'r--','LineWidth',2,'MaxHeadSize',0.5);
quiver3(1,5,2,y_body(1),y_body(2),y_body(3),'g--','LineWidth',2,'MaxHeadSize',0.5);
quiver3(1,5,2,z_body(1),z_body(2),z_body(3),'b--','LineWidth',2,'MaxHeadSize',0.5);
```

Finally we can change some properties of the plot. To do this, we will use the following code:

```matlab
axis equal;
xlim([-10 10]);
ylim([-10 10]);
zlim([-10 10]);
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
```

The final result is the following:




### Subsection 1.1

Phasellus vel mauris eget lorem blandit pharetra ut at velit.

### Subsection 1.2

Aliquam erat volutpat. Vestibulum ultrices gravida lectus, sit amet vestibulum odio facilisis vel.

## Section 2

Etiam non ex eget metus tincidunt suscipit. Sed interdum, sapien sed semper sagittis, eros risus elementum nisl, vitae convallis libero velit a velit.

...
