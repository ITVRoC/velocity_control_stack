#!/usr/bin/env python

'''
Universidade Federal de Minas Gerais (UFMG) - 2022
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Victor R. F. Miranda, <victormrfm@gmail.com>
'''
import sys
import time
import threading, os
import numpy as np
import rospy
from lib.pid_class import PID
from lib.bib_dynamic import dynamic_mode

# messages
from geometry_msgs.msg import Twist, Wrench, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState


## Class running on robot
class robot:
	def __init__(self):
		self.vel = 0.0
		self.vel_ref = 0.0
		self.ang_vel = 0.0
		self.ang_vel_ref = 0.0
		# KP = 1.0e+03*1.2256
		# KI = 1.0e+03*1.5828
		# KD = 2.1452
		KP = rospy.get_param('robust_control/pid_controller/kp')
		KI = rospy.get_param('robust_control/pid_controller/ki')
		KD = rospy.get_param('robust_control/pid_controller/kd')
		TS = rospy.get_param('robust_control/pid_controller/ts')
		ETOL = rospy.get_param('robust_control/pid_controller/etol')
		self.controlador = PID(KP,KI,KD,TS,ETOL)
		# KP = 63.3
		# KP = 1.0e+03*0.0564
		# # KI = 1.0e+03*0.3187
		# KI = 1.0e+03*0.4187
		# KD = 32.4738
		KP = rospy.get_param('robust_control/pid_controller/ori_kp')
		KI = rospy.get_param('robust_control/pid_controller/ori_ki')
		KD = rospy.get_param('robust_control/pid_controller/ori_kd')
		TS = rospy.get_param('robust_control/pid_controller/ori_ts')
		ETOL = rospy.get_param('robust_control/pid_controller/ori_etol')
		self.controlador2 = PID(KP,KI,KD,TS,ETOL)

		self.dynamics = dynamic_mode()

		odom_topic = rospy.get_param('robust_control/topic_name')
		rospy.init_node('PID_Control', anonymous=True)
		rospy.Subscriber(odom_topic, Odometry, self.callback_velocity)
		rospy.Subscriber("/cmd_vel", Twist, self.callback_reference)

		# Creating Publishers
		self.motor_publisher1 = rospy.Publisher('/device1/set_joint_state', JointState, queue_size=10)
		self.motor_publisher2 = rospy.Publisher('/device2/set_joint_state', JointState, queue_size=10)
		self.motor_publisher3 = rospy.Publisher('/device3/set_joint_state', JointState, queue_size=10)
		self.motor_publisher4 = rospy.Publisher('/device4/set_joint_state', JointState, queue_size=10)
		self.motor_publisher5 = rospy.Publisher('/device5/set_joint_state', JointState, queue_size=10)
		self.motor_publisher6 = rospy.Publisher('/device6/set_joint_state', JointState, queue_size=10)

		self.last_time = rospy.Time.now()
		self.new_data = False



	def callback_velocity(self, data):
		beta = 0.3
		self.vel = (1-beta)*self.vel + beta*data.twist.twist.linear.x
		self.ang_vel = data.twist.twist.angular.z


	def callback_reference(self, data):
		self.vel_ref = data.linear.x

		self.ang_vel_ref = data.angular.z

		self.last_time = rospy.Time.now()
		self.new_data = True
		

	def run(self):
		rate = rospy.Rate(rospy.get_param('robust_control/control_frequency'))
		print("\33[92mStarting PID Robust Control in Robot Mode \33[0m")

		t_previous = rospy.Time.now().to_sec()

		while not rospy.is_shutdown():
			if ((rospy.get_rostime() - self.last_time).to_sec() > 1):
				self.vel_ref = 0.0
				self.ang_vel_ref = 0.0

				stop_state = JointState()
				stop_state.header.stamp = rospy.Time.now()
				stop_state.velocity = [0]

				self.motor_publisher1.publish(stop_state)
				self.motor_publisher2.publish(stop_state)
				self.motor_publisher3.publish(stop_state)
				self.motor_publisher4.publish(stop_state)
				self.motor_publisher5.publish(stop_state)
				self.motor_publisher6.publish(stop_state)

			else:
				t_current = rospy.Time.now().to_sec()
				dt = t_current - t_previous
				t_previous = t_current

				self.controlador.Ts = dt
				self.controlador2.Ts = dt

				# Controladores
				u1 = self.controlador.update(self.vel_ref,self.vel)
				u2 = self.controlador2.update(self.ang_vel_ref,self.ang_vel)
				TR = ((u1-u2)/2.0)
				TL = -((u1+u2)/2.0)

				vel_left, vel_right, vel_mid_lef, vel_mid_right = self.dynamics.calc_vel(TL, TR)


				state_right = JointState()
				state_right.header.stamp = rospy.Time.now()
				state_right.velocity = [vel_right]
				
				state_right_middle = JointState()
				state_right_middle.header.stamp = rospy.Time.now()
				state_right_middle.velocity = [vel_mid_right]
				
				state_left = JointState()
				state_left.header.stamp = rospy.Time.now()
				state_left.velocity = [vel_left]
				
				state_left_middle = JointState()
				state_left_middle.header.stamp = rospy.Time.now()
				state_left_middle.velocity = [vel_mid_lef]

				self.motor_publisher1.publish(state_left)
				self.motor_publisher2.publish(state_left_middle)
				self.motor_publisher3.publish(state_left)
				self.motor_publisher4.publish(state_right)
				self.motor_publisher5.publish(state_right_middle)
				self.motor_publisher6.publish(state_right)



			rate.sleep()



########### MAIN #####################
if __name__ == '__main__':
	node = robot()
	node.run()

