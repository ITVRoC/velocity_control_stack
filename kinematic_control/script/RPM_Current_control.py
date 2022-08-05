#!/usr/bin/env python

'''
Universidade Federal de Minas Gerais (UFMG) - 2022
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Victor R. F. Miranda, <victormrfm@gmail.com>
'''

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Int16

from libs.pid_class import PID


rpm1 = 0.0
rpm2 = 0.0
rpm3 = 0.0
rpm4 = 0.0
rpm5 = 0.0
rpm6 = 0.0
 
current1 = 0.0
current6 = 0.0

vel = [0,0]

cmd1 = 0.0
cmd2 = 0.0
cmd3 = 0.0
cmd4 = 0.0
cmd5 = 0.0
cmd6 = 0.0


last_time = rospy.Time()

# Motor velocity callbacks
def motor1_callback(message):
	global rpm1
	rpm1 = message.velocity[0]
	#print("RPM motor 1: %f" % rpm1)

def motor2_callback(message):
	global rpm2
	rpm2 = message.velocity[0]

def motor3_callback(message):
	global rpm3
	rpm3 = message.velocity[0]

def motor4_callback(message):
	global rpm4
	rpm4 = message.velocity[0]

def motor5_callback(message):
	global rpm5
	rpm5 = message.velocity[0]

def motor6_callback(message):
	global rpm6
	rpm6 = message.velocity[0]



def current1_callback(message):
	global current1
	current1 = message.data

def current6_callback(message):
	global current6
	current6 = message.data


def callback_RPM_1(msg):
	global last_time, cmd1
	last_time = rospy.Time.now()

	cmd1 = msg.data

def callback_RPM_2(msg):
	global cmd2
	cmd2 = msg.data

def callback_RPM_3(msg):
	global cmd3
	cmd3 = msg.data

def callback_RPM_4(msg):
	global cmd4
	cmd4 = msg.data

def callback_RPM_5(msg):
	global cmd5
	cmd5 = msg.data

def callback_RPM_6(msg):
	global cmd6
	cmd6 = msg.data



def run():
	global rpm1, rpm2, rpm3, rpm4, rpm5, rpm6
	global cmd1, cmd2, cmd3, cmd4, cmd5, cmd6
	global current1,current6
	global last_time
	rospy.init_node('current_control', anonymous=True)

	subscriber_motor1 = rospy.Subscriber("/device1/get_joint_state", JointState, motor1_callback)
	subscriber_motor2 = rospy.Subscriber("/device2/get_joint_state", JointState, motor2_callback)
	subscriber_motor3 = rospy.Subscriber("/device3/get_joint_state", JointState, motor3_callback)
	subscriber_motor4 = rospy.Subscriber("/device4/get_joint_state", JointState, motor4_callback)
	subscriber_motor5 = rospy.Subscriber("/device5/get_joint_state", JointState, motor5_callback)
	subscriber_motor6 = rospy.Subscriber("/device6/get_joint_state", JointState, motor6_callback)

	subs_current1 = rospy.Subscriber("/device1/get_current_actual_value", Int16, current1_callback)
	subs_current6 = rospy.Subscriber("/device6/get_current_actual_value", Int16, current6_callback)

	sub_rpm1 = rospy.Subscriber("/device1/cmd_rpm",Int16,callback_RPM_1)
	sub_rpm2 = rospy.Subscriber("/device2/cmd_rpm",Int16,callback_RPM_2)
	sub_rpm3 = rospy.Subscriber("/device3/cmd_rpm",Int16,callback_RPM_3)
	sub_rpm4 = rospy.Subscriber("/device4/cmd_rpm",Int16,callback_RPM_4)
	sub_rpm5 = rospy.Subscriber("/device5/cmd_rpm",Int16,callback_RPM_5)
	sub_rpm6 = rospy.Subscriber("/device6/cmd_rpm",Int16,callback_RPM_6)

	motor_publisher1 = rospy.Publisher('/device1/set_joint_state', JointState, queue_size=10)
	motor_publisher2 = rospy.Publisher('/device2/set_joint_state', JointState, queue_size=10)
	motor_publisher3 = rospy.Publisher('/device3/set_joint_state', JointState, queue_size=10)
	motor_publisher4 = rospy.Publisher('/device4/set_joint_state', JointState, queue_size=10)
	motor_publisher5 = rospy.Publisher('/device5/set_joint_state', JointState, queue_size=10)
	motor_publisher6 = rospy.Publisher('/device6/set_joint_state', JointState, queue_size=10)

	

	freq = rospy.get_param('robot_locomotion/control_rate')
	int_erro = 0.0
	dt = 1.0/freq
	

	# PID PARAMETERS
	control1 = PID(0.5,0.1,0.0,dt,0)
	control2 = PID(0.5,0.1,0.0,dt,0) # conferir
	control3 = PID(0.5,0.2,0.0,dt,0)
	control4 = PID(0.5,0.2,0.0,dt,0)
	control5 = PID(0.5,0.1,0.0,dt,0) # conferir
	control6 = PID(0.5,0.4,0.0,dt,0)


	msg1 = JointState()
	msg2 = JointState()
	msg3 = JointState()
	msg4 = JointState()
	msg5 = JointState()
	msg6 = JointState()


	rate = rospy.Rate(freq)
	while not rospy.is_shutdown():
		mode_type = rospy.get_param('/robot_locomotion/operation_mode')

		if mode_type == "current":
		
			u1 = 0
			u2 = 0
			u3 = 0
			u4 = 0
			u5 = 0 
			u6 = 0

			# Only plot
			erro = cmd1 - rpm1

			if ((rospy.get_rostime() - last_time).to_sec() > 1):
				msg1.header.stamp = rospy.Time.now()
				msg1.effort = [0.0]
				msg2.header.stamp = rospy.Time.now()
				msg2.effort = [0.0]
				msg3.header.stamp = rospy.Time.now()
				msg3.effort = [0.0]
				msg4.header.stamp = rospy.Time.now()
				msg4.effort = [0.0]
				msg5.header.stamp = rospy.Time.now()
				msg5.effort = [0.0]
				msg6.header.stamp = rospy.Time.now()
				msg6.effort = [0.0]

			else:		
				u1 = control1.update(cmd1,rpm1)
				msg1.header.stamp = rospy.Time.now()
				msg1.effort = [u1]

				u2 = control2.update(cmd2,rpm2)
				msg2.header.stamp = rospy.Time.now()
				msg2.effort = [u2]

				u3 = control3.update(cmd3,rpm3)
				msg3.header.stamp = rospy.Time.now()
				msg3.effort = [u3]

				u4 = control4.update(cmd4,rpm4)
				msg4.header.stamp = rospy.Time.now()
				msg4.effort = [u4]

				u5 = control5.update(cmd5,rpm5)
				msg5.header.stamp = rospy.Time.now()
				msg5.effort = [u5]

				u6 = control6.update(cmd6,rpm6)
				msg6.header.stamp = rospy.Time.now()
				msg6.effort = [u6]


			
			motor_publisher1.publish(msg1)
			motor_publisher2.publish(msg2)
			motor_publisher3.publish(msg3)
			motor_publisher4.publish(msg4)
			motor_publisher5.publish(msg5)
			motor_publisher6.publish(msg6)

			# print("U1 := ", u1)
			# print("U2 := ", u2)
			# print("\n")

			# print("Reference = %f" % reference)
			# print("RPM 6 = %f" % rpm6)
			# print("Erro = %f" % erro)
			# print("Current = %f" % current6)
			# print("Control = %f" % u6)
			# print("\n")

		rate.sleep()

	print("FIM")


if __name__ == '__main__':
	run()
	

	