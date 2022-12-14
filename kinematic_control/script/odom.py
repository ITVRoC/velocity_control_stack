#!/usr/bin/env python

'''
Universidade Federal de Minas Gerais (UFMG) - 2022
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Victor R. F. Miranda, <victormrfm@gmail.com>
'''

from math import sin, cos, pi

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState


from libs.bib_robot_skid import Robot_skid 

class odometry:
    def __init__(self):
        # Robot Pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.Dth = 0.0

        # Motor velocities
        self.motor_velocity1 = 0
        self.motor_velocity2 = 0
        self.motor_velocity3 = 0
        self.motor_velocity4 = 0
        self.motor_velocity5 = 0
        self.motor_velocity6 = 0

	# Robot mode
        self.wheels_mode = rospy.get_param('robot_locomotion/number_wheels')

        # Wheel radius for TF (base_init > chassis_init)
        self.wheel_radius = rospy.get_param('robot_locomotion/wheeled_radius')

        self.time_counter_aux = 0
        self.ros_init()

    def ros_init(self):

        rospy.init_node('wheel_odometry_publisher', anonymous=True)
        rospy.loginfo("Computing Wheel Odometry")

        motor_config = rospy.get_param('robot_locomotion/motor_config')

        # Times used to integrate velocity to pose
        self.current_time = 0.0
        self.last_time = 0.0

        # Create subscribers that receives the wheels velocities
        # UFMG MODE
        if(motor_config == 1):
            self.subscriber_motor1 = rospy.Subscriber("/device1/get_joint_state", JointState, self.motor1_callback)
            self.subscriber_motor2 = rospy.Subscriber("/device2/get_joint_state", JointState, self.motor2_callback)
            self.subscriber_motor3 = rospy.Subscriber("/device3/get_joint_state", JointState, self.motor3_callback)
            self.subscriber_motor4 = rospy.Subscriber("/device4/get_joint_state", JointState, self.motor4_callback)
            self.subscriber_motor5 = rospy.Subscriber("/device5/get_joint_state", JointState, self.motor5_callback)
            self.subscriber_motor6 = rospy.Subscriber("/device6/get_joint_state", JointState, self.motor6_callback)
        # ITV MODE
        else:
            self.subscriber_motor1 = rospy.Subscriber("/device6/get_joint_state", JointState, self.motor1_callback)
            self.subscriber_motor2 = rospy.Subscriber("/device5/get_joint_state", JointState, self.motor2_callback)
            self.subscriber_motor3 = rospy.Subscriber("/device4/get_joint_state", JointState, self.motor3_callback)
            self.subscriber_motor4 = rospy.Subscriber("/device3/get_joint_state", JointState, self.motor4_callback)
            self.subscriber_motor5 = rospy.Subscriber("/device2/get_joint_state", JointState, self.motor5_callback)
            self.subscriber_motor6 = rospy.Subscriber("/device1/get_joint_state", JointState, self.motor6_callback)

        # odom publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.vel_pub = rospy.Publisher("robot_vel", Twist, queue_size=1)

        # Tf broadcaster
        self.odom_broadcaster = tf.TransformBroadcaster()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    # Motor velocity callbacks
    def motor1_callback(self, message):
        self.motor_velocity1 = message.velocity[0]

    def motor2_callback(self, message):
        self.motor_velocity2 = message.velocity[0]

    def motor3_callback(self, message):
        self.motor_velocity3 = message.velocity[0]

    def motor4_callback(self, message):
        self.motor_velocity4 = message.velocity[0]

    def motor5_callback(self, message):
        self.motor_velocity5 = message.velocity[0]

    def motor6_callback(self, message):
        self.motor_velocity6 = message.velocity[0]
        self.current_time = message.header.stamp.secs + message.header.stamp.nsecs*0.000000001

        #self.odometry_calculation()
        if self.last_time > 0.0:
            self.odometry_calculation()
        else:
            self.last_time = self.current_time

    def odometry_calculation(self):
        robot = Robot_skid(rospy.get_param('/robot_locomotion/'))
        ### Skid-Steering model
        v_robot, w_robot = robot.get_kinematic_velocity([self.motor_velocity1,  self.motor_velocity2,  self.motor_velocity3,  self.motor_velocity4,  self.motor_velocity5,  self.motor_velocity6])

        if self.time_counter_aux == 0:
            self.last_time = self.current_time
            self.time_counter_aux = 1

        # Velocity in the XY plane
        vx_robot = v_robot * cos(self.th)
        vy_robot = v_robot * sin(self.th)

        # Calculating odometry
        dt = self.current_time - self.last_time
        delta_x = vx_robot * dt
        delta_y = vy_robot * dt
        delta_th = w_robot * dt

        # Integrating pose
        self.x += delta_x
        self.y += delta_y

        self.th += delta_th


        vel_robot = Twist()
        vel_robot.linear.x = v_robot
        vel_robot.angular.z = w_robot

        # Since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # Transform from base_init to chassis_init
        self.odom_broadcaster.sendTransform(
            (0., 0., self.wheel_radius),
            (0., 0., 0., 1.),
            rospy.Time.from_sec(self.current_time), # rospy.Time.now()
            "chassis_init", # base_link
            "base_init", # odom
        )
        
        # First, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            rospy.Time.from_sec(self.current_time), # rospy.Time.now()
            "wheel_odom", # base_link
            "chassis_init", # odom
        )
        
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = rospy.Time.from_sec(self.current_time) # rospy.Time.now()
        odom.header.frame_id = "chassis_init" # odom

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "wheel_odom" # base_link
        odom.twist.twist = Twist(Vector3(v_robot, 0, 0), Vector3(0, 0, w_robot))

        # Calculating the covariance based on the angular velocity
        # if the robot is rotating, the covariance is higher than if its going straight
        if abs(w_robot) > 0.2:
            covariance_cons = 0.3
        else:
            covariance_cons = 0.05

        odom.pose.covariance[0] = covariance_cons
        odom.pose.covariance[7] = covariance_cons
        odom.pose.covariance[35] = 100 * covariance_cons

        odom.pose.covariance[1] = covariance_cons
        odom.pose.covariance[6] = covariance_cons

        odom.pose.covariance[31] = covariance_cons
        odom.pose.covariance[11] = covariance_cons

        odom.pose.covariance[30] = 10 * covariance_cons
        odom.pose.covariance[5] = 10 * covariance_cons

        odom.pose.covariance[14] = 0.1
        odom.pose.covariance[21] = 0.1
        odom.pose.covariance[28] = 0.1

        odom.twist.covariance[0] = covariance_cons
        odom.twist.covariance[7] = covariance_cons
        odom.twist.covariance[35] = 100 * covariance_cons

        odom.twist.covariance[1] = covariance_cons
        odom.twist.covariance[6] = covariance_cons

        odom.twist.covariance[31] = covariance_cons
        odom.twist.covariance[11] = covariance_cons

        odom.twist.covariance[30] = 10 * covariance_cons
        odom.twist.covariance[5] = 10 * covariance_cons

        odom.twist.covariance[14] = 0.1
        odom.twist.covariance[21] = 0.1
        odom.twist.covariance[28] = 0.1

        # publish the message
        self.odom_pub.publish(odom)
        self.vel_pub.publish(vel_robot)

        self.last_time = self.current_time


if __name__ == '__main__':
    odometry_obj = odometry()
