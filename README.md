# Workpiece_3D_reconstruction
Reconstruct the 3D model for workpiece by robot UR10

## 1. Prerequisites
* In order to complete this tutorial, you should have access to the following:
  * A computer with ROS installed (>= Hydro)
  * ROS-Industrial's universal_robot package

## 2. Steps
* Install universal_robot package from: http://wiki.ros.org/universal_robot
* Connect the Ethernet and set the ip address as 192.168.1.x. ping 192.168.1.10 to check whether the first robot is connected
* The robot in the lab is UR v3.x. So we need to install UR modern driver from: https://github.com/ThomasTimm/ur_modern_driver 
Note:  swapping every instance of `->hardware_interface` for `->claimed_resources.at(0).hardware_interface` in ur_modern_driver/src/ur_hardware_interface.cpp allows the package to build in Kinetic.
To bring up the real robot, run the following command after replacing IP_OF_THE_ROBOT with the actual hardware's address:
```
roslaunch ur_modern_driver ur10_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]
```

## 3. Using MoveIt! with hardware
Note that as MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:
```
roslaunch ur_modern_driver ur10_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch limited:=true
roslaunch ur10_moveit_config moveit_rviz.launch config:=true
```

This is modified from:
http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial

## 3. Using Gazebo
Follow the instructions below, we can easily manipulate the UR robot in virtual world!
modified from http://wiki.ros.org/ur_gazebo
To launch the simulated arm and a controller for it, run:
```
roslaunch ur_gazebo ur10.launch
```
and in another terminal:

```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
```
To control the simulated arm from RViz, also run:

```
roslaunch ur10_moveit_config moveit_rviz.launch config:=true
```
You should now be able to move the end effector goal to create a plan for the simulated arm to execute.

## 4 . Attach a RGBD camera to UR10 robot
The model of UR10 robot is written in URDF files(older version compared with SDF files)
These are their differences.http://answers.gazebosim.org/question/62/sdf-vs-urdf-what-should-one-use/
There is also method to convert URDF to SDF.http://answers.gazebosim.org/question/7074/urdf-to-sdf-conversion-using-gzsdf/

## 5 . Octomap
We can output the point cloud to an Octomap for environment reconstruction
```
roslaunch octomap_server test_octomap.launch
```
