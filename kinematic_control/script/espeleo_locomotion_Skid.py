#!/usr/bin/env python

'''
Universidade Federal de Minas Gerais (UFMG) - 2022
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Victor R. F. Miranda, <victormrfm@gmail.com>
'''


import rospy
import dynamic_reconfigure.client

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Wrench
from std_msgs.msg import Int16

from libs.bib_espeleo_skid import Espeleo_skid 



def dyn_config_callback(cfg):
    forward_orientation = rospy.get_param('/espeleo_locomotion/forward_orientation') 

    if "forward_orientation" in cfg.keys():
        forward_orientation = cfg["forward_orientation"]
        print(cfg)


class Espeleo_locomotion():

    # constructor class
    def __init__(self):
        
        # initilializing node
        rospy.init_node('espeleo_locomotion', anonymous=True, log_level=rospy.DEBUG)

         # trying to get robot parameters from ROS parameter server
        try:
            self.locomotionParamDictionary = rospy.get_param('/espeleo_locomotion/')     
        except:
            
            # sends a fatal message to the user
            rospy.logfatal("Espeleo default parameters not found on ROS param server. Check if ./espeleo_locomotion/config/locomotion_parameters.yaml file exists, and if it is being correctly loaded into a launch file. Shutting down the node")

            # shuts down the node
            rospy.signal_shutdown('Shutting down the node.')
            return None

        # sending a message
        rospy.loginfo('Espeleo_Locomotion 3.0 node initiated.')

        # Creating Subscribers
        rospy.Subscriber("cmd_vel", Twist, self.callback_command)
        # rospy.Subscriber("/operation_mode", Int16, self.callback_mode)
        # rospy.Subscriber("/cmd_torque", Wrench, self.callback_torque)

        # Creating Publishers
        self.motor_publisher1 = rospy.Publisher('/device1/set_joint_state', JointState, queue_size=10)
        self.motor_publisher2 = rospy.Publisher('/device2/set_joint_state', JointState, queue_size=10)
        self.motor_publisher3 = rospy.Publisher('/device3/set_joint_state', JointState, queue_size=10)
        self.motor_publisher4 = rospy.Publisher('/device4/set_joint_state', JointState, queue_size=10)
        self.motor_publisher5 = rospy.Publisher('/device5/set_joint_state', JointState, queue_size=10)
        self.motor_publisher6 = rospy.Publisher('/device6/set_joint_state', JointState, queue_size=10)

        # RPM Publisher to Current COntrol
        self.rpm_publisher1 = rospy.Publisher('/device1/cmd_rpm', Int16, queue_size=1)
        self.rpm_publisher2 = rospy.Publisher('/device2/cmd_rpm', Int16, queue_size=1)
        self.rpm_publisher3 = rospy.Publisher('/device3/cmd_rpm', Int16, queue_size=1)
        self.rpm_publisher4 = rospy.Publisher('/device4/cmd_rpm', Int16, queue_size=1)
        self.rpm_publisher5 = rospy.Publisher('/device5/cmd_rpm', Int16, queue_size=1)
        self.rpm_publisher6 = rospy.Publisher('/device6/cmd_rpm', Int16, queue_size=1)


        # Create Lib Object
        self.espeleo_obj = Espeleo_skid(self.locomotionParamDictionary)

        # Variables
        self.wheels_velocity = [0, 0, 0, 0, 0, 0]
        self.last_time = rospy.Time()



    def callback_command(self, data):
        cmd_vel_x = data.linear.x
        w = data.angular.z

        if not self.locomotionParamDictionary['forward_orientation']:
            cmd_vel_x = cmd_vel_x * (-1)

        self.last_time = rospy.Time.now()

        self.wheels_velocity = self.espeleo_obj.compute_kinematicMotion(cmd_vel_x, w)



    def run(self):
        rate = rospy.Rate(self.locomotionParamDictionary['control_rate'])
        mode_type = self.locomotionParamDictionary['operation_mode']
        motor_config = self.locomotionParamDictionary['motor_config']

        while not rospy.is_shutdown():
            # mode_type = rospy.get_param('/kacanopen_espeleo/operation_mode')
            
            #################################################################
            ################ Velocity  Mode #################################
            #################################################################
            if mode_type == "velocity":
                state_right = JointState()
                state_right.header.stamp = rospy.Time.now()
                state_right.velocity = [self.wheels_velocity[3]]
                
                state_right_middle = JointState()
                state_right_middle.header.stamp = rospy.Time.now()
                state_right_middle.velocity = [self.wheels_velocity[4]]
                
                state_left = JointState()
                state_left.header.stamp = rospy.Time.now()
                state_left.velocity = [self.wheels_velocity[0]]
                
                state_left_middle = JointState()
                state_left_middle.header.stamp = rospy.Time.now()
                state_left_middle.velocity = [self.wheels_velocity[1]]
                    

                if ((rospy.get_rostime() - self.last_time).to_sec() > 2):
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
                    # UFMG MODE
                    if(motor_config == 1):
                        self.motor_publisher1.publish(state_left)
                        self.motor_publisher2.publish(state_left_middle)
                        self.motor_publisher3.publish(state_left)
                        self.motor_publisher4.publish(state_right)
                        self.motor_publisher5.publish(state_right_middle)
                        self.motor_publisher6.publish(state_right)
                    # ITV MODE
                    else:
                        self.motor_publisher6.publish(state_left)
                        self.motor_publisher5.publish(state_left_middle)
                        self.motor_publisher4.publish(state_left)
                        self.motor_publisher3.publish(state_right)
                        self.motor_publisher2.publish(state_right_middle)
                        self.motor_publisher1.publish(state_right)

            #################################################################
            ################# Current  Mode #################################
            #################################################################
            elif mode_type == "current":
                if ((rospy.get_rostime() - self.last_time).to_sec() > 2):
                    stop_state = Int16()
                    stop_state.data = 0.0

                    self.rpm_publisher1.publish(stop_state)
                    self.rpm_publisher2.publish(stop_state)
                    self.rpm_publisher3.publish(stop_state)
                    self.rpm_publisher4.publish(stop_state)
                    self.rpm_publisher5.publish(stop_state)
                    self.rpm_publisher6.publish(stop_state)
                else:
                    state_right = Int16()
                    state_right.data = self.wheels_velocity[3]

                    state_right_middle = Int16()
                    state_right_middle.data = self.wheels_velocity[4]

                    state_left = Int16()
                    state_left.data = self.wheels_velocity[0]

                    state_left_middle = Int16()
                    state_left_middle.data = self.wheels_velocity[1]

                    # UFMG MODE
                    if(motor_config == 1):
                        #left
                        self.rpm_publisher1.publish(state_left)
                        self.rpm_publisher2.publish(state_left_middle)
                        self.rpm_publisher3.publish(state_left)
                        #right
                        self.rpm_publisher4.publish(state_right)
                        self.rpm_publisher5.publish(state_right_middle)
                        self.rpm_publisher6.publish(state_right)

                    # ITV MODE
                    else:
                        #left
                        self.rpm_publisher6.publish(state_left)
                        self.rpm_publisher5.publish(state_left_middle)
                        self.rpm_publisher4.publish(state_left)
                        #right
                        self.rpm_publisher3.publish(state_right)
                        self.rpm_publisher2.publish(state_right_middle)
                        self.rpm_publisher1.publish(state_right)



            rate.sleep()





#================= starts the node ========================
if __name__ == '__main__':
    client = dynamic_reconfigure.client.Client("espeleo", timeout=5, config_callback=dyn_config_callback)
    espeleo = Espeleo_locomotion()
    espeleo.run()

