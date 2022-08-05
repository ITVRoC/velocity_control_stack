#!/usr/bin/env python

'''
Universidade Federal de Minas Gerais (UFMG) - 2022
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Victor R. F. Miranda, <victormrfm@gmail.com>
'''

import rospy
import numpy as np
import math
  
# Class definition
class Robot_skid:
    
    # class constructor
    def __init__(self, locomotionParamDictionary):

        # --- saving the locomotion dictionary into a local variable
        self.locomotionParamDictionary = locomotionParamDictionary

    # Set max rpm
    def rpm_check_saturation(self,rpm):
        max_rpm = self.locomotionParamDictionary['motor_rpm_saturation']
        if(rpm >= max_rpm):
            rpm = max_rpm
        elif(rpm <= -max_rpm):
            rpm = -max_rpm
        
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
    def extremity_wheels_to_motor(self, wheel_rpm):
        motor_rpm = wheel_rpm * (self.locomotionParamDictionary['planetary_gear_reduction'] * self.locomotionParamDictionary['extremity_wheel_reduction'])   
        return motor_rpm

    # Transform Wheel RPM to motor RPM
    def central_wheels_to_motor(self, wheel_rpm):
        motor_rpm = wheel_rpm * (self.locomotionParamDictionary['planetary_gear_reduction'] * self.locomotionParamDictionary['central_wheel_reduction'])
        return motor_rpm

    def motor_to_extremity_wheels(self, motor_rpm):
        external_wheel = motor_rpm / (self.locomotionParamDictionary['planetary_gear_reduction'] * self.locomotionParamDictionary['extremity_wheel_reduction'])
        return external_wheel

    def motor_to_central_wheels(self, motor_rpm):
        internal_wheel = motor_rpm / (self.locomotionParamDictionary['planetary_gear_reduction'] * self.locomotionParamDictionary['central_wheel_reduction'])
        return internal_wheel


    def left_right_velocity(self, motor_velocities):

        # Takes motor velocities and transforms then into wheel velocity, according to their position (internal or external)
        wheel_1 = self.motor_to_extremity_wheels(motor_velocities[0])
        wheel_2 = self.motor_to_central_wheels(motor_velocities[1])
        wheel_3 = self.motor_to_extremity_wheels(motor_velocities[2])
        wheel_4 = self.motor_to_extremity_wheels(motor_velocities[3])
        wheel_5 = self.motor_to_central_wheels(motor_velocities[4])
        wheel_6 = self.motor_to_extremity_wheels(motor_velocities[5])
        wheels = [wheel_1, wheel_2, wheel_3, wheel_4, wheel_5, wheel_6]

        # Transforms RPM to rad/s
        wheels = self.rpm_to_rad(wheels)

        # Acording to wheels configuration, calculate the linear and angular velocity
        if self.locomotionParamDictionary['number_wheels'] == 4:
            velocity_left = [wheels[0], wheels[2]]
            velocity_right = [wheels[3], wheels[5]]
            # Calculate right and left velocitys, considering that when the move fowards, the motor on the left has a positive velocity, and the motors on the right has negative.
            velocity_left = sum(velocity_left) / len(velocity_left)
            velocity_right = (sum(velocity_right) / len(velocity_right)) * -1

        elif self.locomotionParamDictionary['number_wheels'] == 6:
            velocity_left = [wheels[0], wheels[1], wheels[2]]
            velocity_right = [wheels[3], wheels[4], wheels[5]]

        
        return (velocity_right, velocity_left)


    def get_kinematic_velocity(self, motor_velocities):

        velocity_right, velocity_left = self.left_right_velocity(motor_velocities)

        r = self.locomotionParamDictionary['wheeled_radius']
        a = self.locomotionParamDictionary['a']
        b = self.locomotionParamDictionary['b']
        L = self.locomotionParamDictionary['L']

        if self.locomotionParamDictionary['number_wheels'] == 4:
            M = np.asarray([[0.5, 0.5],[b/(2*(a**2 + b**2)), -b/(2*(a**2 + b**2))]])
            vel = np.dot(M,np.asarray([[velocity_right],[velocity_left]]))

            vel = vel*r

            v = vel[0][0]
            w = vel[1][0]
            
        elif self.locomotionParamDictionary['number_wheels'] == 6:
            vd = (velocity_right[0] + velocity_right[2]) / 2.0 * -1.0
            ve = ((velocity_left[0] + velocity_left[2]) / 2.0) 
            VD = velocity_right[1] * -1.0
            VE = velocity_left[1] 

            # Normal Wheels
            M = np.asarray([[0.5, 0.5],[b/(2*(a**2 + b**2)), -b/(2*(a**2 + b**2))]])
            vel = np.dot(M,np.asarray([[vd],[ve]]))
            vel = vel*r

            # print("Vel := ", vel)

            # Middle Wheels
            N = np.asarray([[0.5, 0.5],[1/(2*L), -1/(2*L)]])
            V_vec = np.dot(N,np.asarray([[VD],[VE]]))
            V_vec = V_vec*r

            # print("V := ", V_vec)

            v = (2.0/3.0)*vel[0][0] + (1.0/3.0)*V_vec[0][0]
            w = (2.0/3.0)*vel[1][0] + (1.0/3.0)*V_vec[1][0]
            

        return v, w


    def compute_kinematicMotion(self, v, w):
        r = self.locomotionParamDictionary['wheeled_radius']
        a = self.locomotionParamDictionary['a']
        b = self.locomotionParamDictionary['b']
        L = self.locomotionParamDictionary['L']


        # Compute Inverse Kinematic Model
        if self.locomotionParamDictionary['number_wheels'] == 4:

            M = np.asarray([[0.5, 0.5],[b/(2*(a**2 + b**2)), -b/(2*(a**2 + b**2))]])
            M_inv = np.linalg.inv(M)
            vel_vec = np.dot(M_inv,np.asarray([[v],[w]]))
            velocity_right = vel_vec[0][0]/r
            velocity_left = vel_vec[1][0]/r


            # Convert rad/s to rpm
            right_wheel_rpm = self.rad_to_rpm(velocity_right)
            left_wheel_rpm = self.rad_to_rpm(velocity_left)


            # Compute RPM velocity considering the reduction gears
            wheel_1 = self.rpm_check_saturation(self.extremity_wheels_to_motor(left_wheel_rpm))
            # wheel_2 = self.rpm_check_saturation(self.central_wheels_to_motor(left_wheel_rpm))
            wheel_3 = self.rpm_check_saturation(self.extremity_wheels_to_motor(left_wheel_rpm))
            wheel_4 = self.rpm_check_saturation(self.extremity_wheels_to_motor(right_wheel_rpm))
            # wheel_5 = self.rpm_check_saturation(self.central_wheels_to_motor(right_wheel_rpm))
            wheel_6 = self.rpm_check_saturation(self.extremity_wheels_to_motor(right_wheel_rpm))

            wheels = [wheel_1, 0, wheel_3, -wheel_4, 0, -wheel_6]

        elif self.locomotionParamDictionary['number_wheels'] == 6:
            # Normal Wheels
            M = np.asarray([[0.5, 0.5],[b/(2*(a**2 + b**2)), -b/(2*(a**2 + b**2))]])
            M_inv = np.linalg.inv(M)
            vel_vec = np.dot(M_inv,np.asarray([[v],[w]]))
            velocity_right = vel_vec[0][0]/r
            velocity_left = vel_vec[1][0]/r


            # Middle Wheels
            N = np.asarray([[0.5, 0.5],[1/(2*L), -1/(2*L)]])
            N_inv = np.linalg.inv(N)
            V_vec = np.dot(N_inv,np.asarray([[v],[w]]))
            VD = V_vec[0][0]/r
            VE = V_vec[1][0]/r


            # Convert rad/s to rpm
            right_wheel_rpm = self.rad_to_rpm(velocity_right)
            right_middle_wheel_rpm = self.rad_to_rpm(VD)
            left_wheel_rpm = self.rad_to_rpm(velocity_left)
            left_middle_wheel_rpm = self.rad_to_rpm(VE)


            # Compute RPM velocity considering the reduction gears
            wheel_1 = self.rpm_check_saturation(self.extremity_wheels_to_motor(left_wheel_rpm))
            wheel_2 = self.rpm_check_saturation(self.central_wheels_to_motor(left_middle_wheel_rpm))
            wheel_3 = self.rpm_check_saturation(self.extremity_wheels_to_motor(left_wheel_rpm))
            wheel_4 = self.rpm_check_saturation(self.extremity_wheels_to_motor(right_wheel_rpm))
            wheel_5 = self.rpm_check_saturation(self.central_wheels_to_motor(right_middle_wheel_rpm))
            wheel_6 = self.rpm_check_saturation(self.extremity_wheels_to_motor(right_wheel_rpm))

            wheels = [wheel_1, wheel_2, wheel_3, -wheel_4, -wheel_5, -wheel_6]

        else:
            # sends a fatal message to the user
            rospy.logfatal("Robot configuration accepts only 4 or 6 wheels. Change the number_wheels parameter in ./kinematic_control/config/locomotion_parameters.yaml")

            # shuts down the node
            rospy.signal_shutdown('Shutting down the node.')


        return wheels

