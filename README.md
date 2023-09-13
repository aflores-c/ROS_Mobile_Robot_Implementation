# ROS_Mobile_Robot_Implementation
 Implementation from scratch of a mobile robot

## Requierements

1. Ubuntu 18 with ROS Melodic installed. This computer can be a laptop, like in this case, but it also can run in an embedded system like a Jetson Nano or a Raspberry. 
2. A differential drive robot with 4 wheels (this project was implemented with 4), but it also can be with two wheels.
3. The hardware: encoders in the motors for the wheels, Arduino Leonardo, Laptop, Two Motor Drivers for 4 motos, RPLIDAR.

## Usage 

You need to implement the Arduino code mobile_robot_final.ino . You need to connect the encoders, motor drivers and serial cable to the Arduino (connects to the computer) 

After that you can implement the robot package in your computer. 
You need to download the repository and paste the requiered package in your workspace, depending on the tutorial you are working.
1. Create a catkin workspace
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
2. Download the repository insider the /src file of the workspace
```sh
$ cd /src
$ copy the mrobot_ws in the src folder
```
3. Make again and check all compiles correctly.
```sh
$ cd ..
$ catkin_make
``` 
## Robot description

#### Overview
You can see the hardware that is used. The connections can be obtained from the Arduino code. In the future, there will be a more detailed connection descrition. 

[robot]: ./images/robot1.png
[running]: ./images/robot_running.jpeg
[mapping]: ./images/mapping.png
[solidworks]: ./images/design.png
[rviz_mapping]: ./images/ros_mapping_rviz.png
[rviz_navigation]: ./images/ros_navigation_rviz.png

![alt text][robot]
![alt text][running]
![alt text][mapping]
![alt text][solidworks]
#### Mapping 
1. Source your environment
```sh
$ cd catkin_ws
$ cd source devel/setup.bash
```
2. Launch the hardware interface. This communicates the Arduino and the Computer with ROS
```sh
$ roslaunch hardware_interface mobile_robot_serial.launch
```
3. Launch the mapping package
```sh
$ roslaunch mobile_robot_slam mobile_robot_slam.launch
```
4. Launch the teleoperation
```sh
$ roslaunch mobile_robot_teleop mobile_robot_teleop_key.launch
```
![alt text][rviz_mapping]

Stop the mapping and place the robot in the initial position where it started mapping
#### Navigation 

1. Launch the hardware interface. This communicates the Arduino and the Computer with ROS
```sh
$ roslaunch hardware_interface mobile_robot_serial.launch
```
2. In another terminal, run the bringup launch file, wich publish the /odom topic and connects to the RPLIDAR
```sh
$ roslaunch hardware_interface mobile_robot_bringup.launch
```
4. In another terminal, run the navigation package
```sh
$ roslaunch mobile_robot_navigation mobile_robot_navigation.launch
```

Inside RVIZ you can start navigating

![alt text][rviz_navigation]
