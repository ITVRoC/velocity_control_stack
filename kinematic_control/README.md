# kinematic_control

This package contains a velocity controller designed using the inverse kinematic model of the wheeled robot. Basically, the package will consider the robot's geometric parameters to define correctly the angular velocity required for each wheel in order to navigate in a defined linear and angular velocity.

This package works in robots with differential or skid-steering geometry. In the last case, we have defined two configurations, 4-wheels or 6-wheels. However, the software can be easily modified for other configuration styles.

## Controller Design Explanation

<p align='center'>
	<img src="/kinematic_control/images/robot_defs_3d.png" alt="center" width="500"/>
	<img src="/kinematic_control/images/robot_defs.png" alt="center" width="300"/>
</p>
<p align='center'>
	Figure: Four-wheeled robot geometry illustration.
</p>


Considering a skid-steering geometry with four wheels shown in the Figure above, we can obtain the kinematic model by analyzing the robot geometry and assuming some nonholonomic constraints. The following equation represents the kinematic model:

<p align='center'>
	<img src="/kinematic_control/images/eq_1.png" alt="center" width="250"/>
</p>

Now, considering that the angular velocity performed by each wheel on the right side is the same, as well as on the left side, we can conclude that the robot linear motion results from the sum of the linear speed performed by each wheel. For the angular motion, considering the anti-symmetry principle, to obtain equilibrium the angular velocity on the right side is the same of the left side, but rotating in the opposite direction. Besides that, the condition for stabilization regards the linear velocity performed by each wheel and a parametric constant based on the robot geometry. Thus, combining linear and angular motion, the kinematic model results in the following equation:

<p align='center'>
	<img src="/kinematic_control/images/eq_2.png" alt="center" width="350"/>
</p>

## ROS - Topics in use

The package subscribes to a ROS message of type [Geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) and publishes to a ROS message [Sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html) for each wheel of the robot. Besides that, the package also publishes to an [Nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) that includes data of the robot's position, velocity, and orientation obtained from the encoders on wheels, subscribing from a [Sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html).

### Subscrribe Topics:
```
/cmd_vel --- Geometry_msgs/Twist
/deviceX/get_joint_state --- Sensor_msgs/JointState , Where X is the wheel number (1,2,3,...,6)
```

### Publish Topics:
```
/deviceX/set_joint_state --- Sensor_msgs/JointState , Where X is the wheel number (1,2,3,...,6)
/odom --- Nav_msgs/Odometry
```
