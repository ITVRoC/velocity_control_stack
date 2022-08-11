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


Considering a skid-steering geometry with four wheels shown in the Figure above, we can obtain the kinematic model by analyzing the robot geometry and assuming some nonholonomic constraints.


## ROS - Topics in use
