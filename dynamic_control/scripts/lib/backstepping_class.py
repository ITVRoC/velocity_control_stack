'''
Universidade Federal de Minas Gerais (UFMG) - 2022
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Victor R. F. Miranda, <victormrfm@gmail.com>
'''

from __future__ import division
import numpy as np
import math
import rospy


####### Function Compute Derivative ##########
def derivative(Ts,x0,x1):
	return ((x1-x0)/Ts)


##### Orientation Control Class ########
class Control:
	def __init__(self, k, k2, Tz, etol):
		self.K = k
		self.K2 = k2
		self.Ts = Tz
		self.etol = etol

		self.last_setpoint = np.array([[0.0],[0.0]])
		self.d_setpoint = np.array([[0.0],[0.0]])

		self.int_error_v = 0.0
		self.int_error_yaw_rate = 0.0
		self.last_error_v = 0.0
		self.last_error_yaw_rate = 0.0


        #### Parameters
		self.m = rospy.get_param('robot_parameters/dynamics/mass')						# mass (kg)
		self.Ca = rospy.get_param('robot_parameters/dynamics/Ca')
		self.Adr = rospy.get_param('robot_parameters/dynamics/Adr')
		self.lf = rospy.get_param('robot_parameters/dynamics/lf')
		self.lr = rospy.get_param('robot_parameters/dynamics/lr')
		self.ls = rospy.get_param('robot_parameters/dynamics/ls')
		self.H = rospy.get_param('robot_parameters/dynamics/friction_coef')
		self.g = rospy.get_param('robot_parameters/dynamics/gravity')
		self.Iz = rospy.get_param('robot_parameters/dynamics/inertia_z')
		self.n = rospy.get_param('robot_parameters/dynamics/motor_efficiency')
		self.inclination = 0.0
		self.r = rospy.get_param('robot_parameters/dynamics/wheel_radio')

		self.max_torque = rospy.get_param('robot_parameters/dynamics/torque_max')

###### Update Control Law ##############
	def update(self, ref, velocity, vel_ang, inclination, friction):
		self.inclination = inclination
		self.H = friction
		error_v = ref[0,0] - velocity[0,0]
		# error_yaw_rate = np.sin(ref[1,0] - vel_ang)
		error_yaw_rate = ref[1,0] - vel_ang
		
		if (abs(error_v) < self.etol):
			error_v = 0.0
		if (abs(error_yaw_rate) < 2*self.etol):
			error_yaw_rate = 0.0

		self.int_error_v = self.int_error_v + ((self.last_error_v + error_v)/2)*self.Ts
		self.last_error_v = error_v

		self.int_error_yaw_rate = self.int_error_yaw_rate + ((self.last_error_yaw_rate + error_yaw_rate)/2)*self.Ts
		self.last_error_yaw_rate = error_yaw_rate




		E = np.array([[error_v],[error_yaw_rate]])
		N = np.array([[self.int_error_v],[self.int_error_yaw_rate]])
		K = self.K
		K2 = self.K2
		a1 = -K.dot(N)
		Z1 = E - a1
		signal = (1.0-np.exp(-velocity[0,0]))/(1+np.exp(-velocity[0,0]))
		self.d_setpoint[0] = derivative(self.Ts, self.last_setpoint[0], ref[0,0])
		self.d_setpoint[1] = derivative(self.Ts, self.last_setpoint[1], ref[1,0])
		self.last_setpoint = ref

		if(np.abs(velocity[0,0])>=0.05):

			f1 = velocity[1,0]*vel_ang - (self.Adr/self.m)*np.abs(velocity[0,0])*velocity[0,0] - self.H*self.g*np.cos(self.inclination)*signal - self.d_setpoint[0,0]
			f2 = ((2*self.H*self.Ca*(self.lr-self.lf))/(self.Iz*(velocity[0,0]+0.0001)))*velocity[1,0] - ((2*self.H*(self.lf*self.lf + self.lr*self.lr))/(self.Iz*(velocity[0,0]+0.0001)))*vel_ang - self.d_setpoint[1,0]
		else:
			f1 = 0.0
			f2 = 0.0

		F = np.array([[f1],[f2]])
		D = np.array([[-self.g*np.sin(self.inclination)],[0.0]])
		G = np.array([[2*self.n/(self.m*self.r), 0.0],[0, 2*self.n*self.ls/(self.Iz*self.r)]])

		m1 = K.dot(K) 
		m2 = m1.dot(N)
		G_inv = np.linalg.inv(G)
		M2 = -F-D+K.dot(Z1)-m2-N-K2.dot(Z1)
		un = G_inv.dot(M2)


		if(ref[0,0] == 0 and ref[1,0] == 0):
			un[0,0] = 0.0
			un[1,0 ] = 0.0
			self.int_error_yaw_rate = 0.0
			self.int_error_v = 0.0
		elif(ref[1,0] == 0):
			un[1,0] = 0.0
			self.int_error_yaw_rate = 0.0
		elif(ref[0,0] == 0):
			un[0,0] = 0.0
			self.int_error_v = 0.0
		else:
			un = un


		

		if(un[1,0] >= 2*self.max_torque):		# direita
			un[1,0] = 2*self.max_torque
			#self.int_error_yaw_rate = self.int_error_yaw_rate + (un[1,0]/(2*(4*self.max_torque-2)))*self.Ts
			self.int_error_yaw_rate = self.int_error_yaw_rate - (((self.last_error_yaw_rate + error_yaw_rate)/2)*self.Ts)
		elif(un[1,0] <= -2*self.max_torque):	# esquerda
			un[1,0] = -2*self.max_torque
			#self.int_error_yaw_rate = self.int_error_yaw_rate + (un[1,0]/(2*(4*self.max_torque-2)))*self.Ts
			self.int_error_yaw_rate = self.int_error_yaw_rate - (((self.last_error_yaw_rate + error_yaw_rate)/2)*self.Ts)


		if(un[0,0] >= 2*self.max_torque):		# tras
			un[0,0] = 2*self.max_torque
			#self.int_error_v = self.int_error_v + (un[0,0]/(2*(4*self.max_torque)))*self.Ts
			self.int_error_v = self.int_error_v - (((self.last_error_v + error_v)/2)*self.Ts)
		elif(un[0,0] <= -2*self.max_torque):	# frente
			un[0,0] = -2*self.max_torque
			#self.int_error_v = self.int_error_v + (un[0,0]/(2*(4*self.max_torque)))*self.Ts
			self.int_error_v = self.int_error_v - (((self.last_error_v + error_v)/2)*self.Ts)


		TD = -((un[0,0]-un[1,0])/2.0)
		TE = ((un[0,0]+un[1,0])/2.0)


		return TD, TE
