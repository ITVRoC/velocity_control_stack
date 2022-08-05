#!/usr/bin/env python

'''
Universidade Federal de Minas Gerais (UFMG) - 2022
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Victor R. F. Miranda, <victormrfm@gmail.com>
'''

import rospy
import math


class dynamic_mode:
	def __init__(self):
		self.cw_reduction = rospy.get_param('robot_parameters/motors/central_wheel_reduction')
		self.ew_reduction = rospy.get_param('robot_parameters/motors/extremity_wheel_reduction')
		self.p_reduction = rospy.get_param('robot_parameters/motors/planetary_gear_reduction')
		self.number_wheels = rospy.get_param('robot_parameters/dynamics/number_wheels')
		self.rpm_sat = rospy.get_param('robot_parameters/motors/motor_rpm_saturation')

	def rpm_check_saturation(self,rpm):
		if (rpm >= self.rpm_sat):
			rpm = self.rpm_sat
		elif(rpm <= - self.rpm_sat):
			rpm = -self.rpm_sat

		return rpm

	def rad_to_rpm(self, velocities):
		if isinstance(velocities, list):
			velocities = [(60.0/(2*math.pi)) * x for x in velocities]
		else:
			velocities = (60.0/(2*math.pi)) * velocities
		return (velocities)

	def rpm_to_rad(self, velocities):
		if isinstance(velocities, list):
			velocities = [((2*math.pi)/60.0) * x for x in velocities]
		else:
			velocities = ((2*math.pi)/60.0) * velocities
		return (velocities)

	# Transform Wheel RPM to motor RPM
	def motor_reduction(self, wheel_rpm):
		motor_rpm = wheel_rpm * (self.p_reduction * self.ew_reduction)
		return motor_rpm

	def motor_reduction_extern(self, wheel_rpm):
		motor_rpm = wheel_rpm * (self.p_reduction * self.cw_reduction)
		return motor_rpm



	def calc_vel(self,torque_L, torque_R):
		rpm_right = self.rpm_check_saturation(self.rad_to_rpm(torque_R))
		rpm_left = self.rpm_check_saturation(self.rad_to_rpm(torque_L))

		if (self.number_wheels == 4):		
			vel_lef = self.motor_reduction(rpm_left)
			vel_right = self.motor_reduction(rpm_right)
			vel_mid_lef = 0.0
			vel_mid_right = 0.0
		elif (self.number_wheels == 6):
			vel_lef = self.motor_reduction(rpm_left)
			vel_right = self.motor_reduction(rpm_right)
			vel_mid_lef = self.motor_reduction_extern(rpm_left)
			vel_mid_right = self.motor_reduction_extern(rpm_right)


		return vel_lef, vel_right, vel_mid_lef, vel_mid_right
