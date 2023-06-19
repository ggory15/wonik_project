import numpy as np
import subprocess, signal, os, inspect, time
import cmd
import rospy
import wonik_docking.srv
import wonik_docking.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus as goal_status
import actionlib
from geometry_msgs.msg import Pose


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
        rospy.init_node('simple_docking_scenario')

        self.docking = rospy.ServiceProxy("/auto_docking", wonik_docking.srv.auto_docking,)
        self.ctrl = rospy.ServiceProxy("/custum_ctrl", wonik_docking.srv.custum_ctrl,)
        self.docking_req = wonik_docking.srv.auto_dockingRequest()
        self.docking_resp = wonik_docking.srv.auto_dockingResponse()        
        self.ctrl_req = wonik_docking.srv.custum_ctrlRequest()
        self.ctrl_resp = wonik_docking.srv.custum_ctrlResponse()
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def do_docking(self, arg):
        self.docking_req.stage = 0
        self.docking_req.individual = False
        self.docking_req.docking = True
        self.docking_resp = self.docking(self.docking_req)
        
    def do_undocking(self,arg):
        self.docking_req.stage = 0
        self.docking_req.individual = False
        self.docking_req.docking = False
        self.docking_resp = self.docking(self.docking_req)

    def do_0step(self, arg):
        self.docking_req.stage = 0
        self.docking_req.individual = True
        self.docking_req.docking = True
        self.docking_resp = self.docking(self.docking_req)

    def do_1step(self, arg):
        self.docking_req.stage = 1
        self.docking_req.individual = True
        self.docking_req.docking = True
        self.docking_resp = self.docking(self.docking_req)

    def do_2step(self, arg):
        self.docking_req.stage = 2
        self.docking_req.individual = True
        self.docking_req.docking = True
        self.docking_resp = self.docking(self.docking_req)

    def do_init(self, arg):
        self.reach_goal = MoveBaseGoal()
        self.reach_goal.target_pose.header.frame_id = 'map'
        self.reach_goal.target_pose.pose.position.x = 8.7
        self.reach_goal.target_pose.pose.position.y = 1.174
        self.reach_goal.target_pose.pose.position.z = 0
        self.reach_goal.target_pose.pose.orientation.x = 0
        self.reach_goal.target_pose.pose.orientation.y = 0
        self.reach_goal.target_pose.pose.orientation.z = 0.7071
        self.reach_goal.target_pose.pose.orientation.w = 0.7071

        self.client.send_goal(self.reach_goal)
    
    def do_first_reach(self, arg):
        goal = Pose()
        goal.position.x = 9.7
        goal.position.y = 2.57
        goal.orientation.x = 0
        goal.orientation.y = 0
        goal.orientation.z = 0.7071
        goal.orientation.w = 0.7071   
        self.ctrl_req.pose = goal     
       
        self.ctrl_resp = self.ctrl(self.ctrl_req)

    def do_second_reach(self, arg):
        goal = Pose()
        goal.position.x = 11.7
        goal.position.y = 2.57
        goal.orientation.x = 0
        goal.orientation.y = 0
        goal.orientation.z = 0.7071
        goal.orientation.w = 0.7071   
        self.ctrl_req.pose = goal     
       
        self.ctrl_resp = self.ctrl(self.ctrl_req)


    def do_quit(self, arg):
        return True
    

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()
