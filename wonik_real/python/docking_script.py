import numpy as np
import subprocess, signal, os, inspect, time
import cmd
import rospy
import wonik_docking.srv
import wonik_docking.msg

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
        self.docking_req = wonik_docking.srv.auto_dockingRequest()
        self.docking_resp = wonik_docking.srv.auto_dockingResponse()        
    
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

    def do_quit(self, arg):
        return True
    

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()
