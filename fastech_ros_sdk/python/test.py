#! /usr/bin/python 

import rospy
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Bool
import importlib, pkgutil
import threading
import cmd, sys, os
from fastech_ros_sdk.srv import ServoOn

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rospy.init_node('simple_test')
        self.set_velocity_pub = rospy.Publisher('/fastech_ros_sdk/set_velocity', JointState, queue_size=1)

    

    def do_motor_on(self, arg):
        rospy.wait_for_service('/fastech_ros_sdk/servo_on')
        try:
            simple_srv = rospy.ServiceProxy('/fastech_ros_sdk/servo_on', ServoOn)
            simple_srv(True)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # from subprocess import call
        # call(["rosservice", "call" ,"/fastech_ros_sdk/servo_on", "'state: True'"])

    def do_motor_off(self, arg):
        rospy.wait_for_service('/fastech_ros_sdk/servo_on')
        try:
            simple_srv = rospy.ServiceProxy('/fastech_ros_sdk/servo_on', ServoOn)
            simple_srv(False)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def do_set_velocity(self, arg):
        target_vel = JointState()
        target_vel.velocity.append(0.0)
        target_vel.velocity[0] = 1.0

        if (len(arg) > 0):
            target_vel.velocity[0] = float(arg)
            print (arg)

        self.set_velocity_pub.publish(target_vel)

    def do_quit(self, arg):
        self.do_motor_off(arg)
        return True
        
   
if __name__ == '__main__':
    ControlSuiteShell().cmdloop()