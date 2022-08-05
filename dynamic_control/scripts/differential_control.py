#!/usr/bin/env python

'''
Universidade Federal de Minas Gerais (UFMG) - 2022
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Victor R. F. Miranda, <victormrfm@gmail.com>
'''

import rospy
# library
from lib.backstepping_class import Control
from lib.bib_dynamic import dynamic_mode

# messages
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int16

import numpy as np
 
 
## Class running on robot
class robot:
	def __init__(self):
		self.velocity = np.array([[0.0],[0.0]])
		self.vel_ref = np.array([[0.0],[0.0]])
		self.ang_vel = 0.0
		self.inclination = 0.0
		self.friction = 0.5
		self.vel_max = rospy.get_param('robot_parameters/dynamics/vel_max')
		self.ang_vel_max = rospy.get_param('robot_parameters/dynamics/ang_vel_max')

		self.dynamics = dynamic_mode()

		k = rospy.get_param('robust_control/controller/K')
		k = np.array(k)
		k2 = rospy.get_param('robust_control/controller/K2')
		k2 = np.array(k2)
		Tz = rospy.get_param('robust_control/controller/Ts')
		etol = rospy.get_param('robust_control/controller/etol')
		self.controlador = Control(k, k2, Tz, etol)


		odom_topic = rospy.get_param('robust_control/topic_name')
		rospy.init_node('robust_control', anonymous=True)
		rospy.Subscriber(odom_topic, Odometry, self.callback_velocity)
		rospy.Subscriber("/cmd_vel", Twist, self.callback_reference)
		rospy.Subscriber("/imu_data", Imu, self.callback_imu)

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
		self.velocity[0] = (1-beta)*self.velocity[0] + beta*data.twist.twist.linear.x
		self.velocity[1] = data.twist.twist.linear.y + 0.001
		self.ang_vel = data.twist.twist.angular.z
		
		


	def callback_reference(self, data):
		self.vel_ref[0] = data.linear.x

		self.vel_ref[1] = data.angular.z

		self.last_time = rospy.Time.now()
		self.new_data = True
		

	def callback_imu(self,data):
		self.inclination = data.orientation.y



	def run(self):
		rate = rospy.Rate(rospy.get_param('robust_control/control_frequency'))
		torque_max = rospy.get_param('robot_parameters/dynamics/torque_max')
		print("\33[92mStarting Dynamic Control in Robot Mode \33[0m")

		t_previous = rospy.Time.now().to_sec()

		while not rospy.is_shutdown():

			if ((rospy.get_rostime() - self.last_time).to_sec() > 1):
				self.vel_ref[0] = 0.0
				self.vel_ref[1] = 0.0

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

				# self.controlador.Ts = dt
				TR,TL = self.controlador.update(self.vel_ref,self.velocity,self.ang_vel,self.inclination, self.friction)


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

			# rospy.spin()
			rate.sleep()



########### MAIN #####################
if __name__ == '__main__':
	node = robot()
	node.run()
