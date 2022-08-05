# Velocity Control Stack For Wheeled Mobile Robots

## Description

This ROS stack provides two packages developed for velocity tracking control (linear and angular) of wheeled mobile robots.
One of these packages considers a kinematic model of the robot in two geometric configurations, differential and skid-steering, which can be chosen by modifying the robot's parameters in the configuration file.
The other package uses a dynamic model representation of the differential geometry to provide two different controllers based on the classical PID and on the Adaptive Integral Backstepping technique. 

Both packages require data acquisition of linear and angular velocities of the robot to compute commands for the robot wheels in order to track desired velocities references. According to the adopted motor type, the wheel commands can be of angular velocity or electric current. For electric current, an auxiliary PID controller computes current commands to track the desired wheel velocity according to the current signal feedback for each wheel.

These packages must be used in cascade with a navigation control to follow a reference path. For example, using one of the packages together with [vectorfield_stack](https://github.com/adrianomcr/vectorfield_stack) to follow a desired path.

## Available packages

### dynamic_control
This package contains a nonlinear controller based on the integral backstepping technique. The controller is designed to track desired linear and angular velocities for a weeled robot considering a nonlinear dynamic model. The strategy use lateral and longitudinal dynamics coupled in a MIMO design of the backstepping with an adaptive part for observable parameters, aiming to mitigate nonlinear effects and other external disturbances due to the roughness of the terrain, for example. 

Details for running, dependences and ROS messages on the package [here](dynamic_control)

### kinematic_control
This package uses a well-defined kinematic model that represents a wheeled robot with differential or skid-steering geometry, according to the geometric parameters selected. This control strategy doesn't use a feedback signal to compare and track desired velocities but could be considered for applications with minor external disturbances or nonlinearities. The control system uses the inverse kinematic model to define wheel commands given desired linear and angular velocities.

Details for running, dependences and ROS messages on the package [here](kinematic_control)

## Main Dependences
- [ROS](http://wiki.ros.org/ROS/Installation) (tested with kinetic and melodic)
```
sudo apt install ros-$ROS_DSITRO-dynamic_reconfigure
```
- Python:
```
sudo apt install python-catkin-tools
pip install numpy
```

## Usage
You can use the following commands to download and compile all packages in your favorite ROS workspace.
```
cd ~/catkin_ws/src
git clone https://github.com/victorRFmiranda/mobilerobot_control_stack
cd ..
catkin_build mobilerobot_control_stack
```

Considering three controllers in this stack, you can run the desired launch, remember to check all configuration parameters:
```
cd ~/catkin_ws
source devel/setup.bash
```

### Dynamic Control Backstepping:
```
roslaunch dynamic_control backstepping.launch
```

### Dynamic Control PID:
```
roslaunch dynamic_control pid.launch
```

### Kinematic Control:
```
roslaunch kinematic_control espeleo_locomotion.launch
```
