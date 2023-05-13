import numpy as np
import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Float32MultiArray
import threading
import cmd, sys, os
from std_msgs.msg import Bool
import math
import tf


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
        marker_sub = rospy.Subscriber('/line_markers', Marker, self.marker_callback)
        self.marker_pub = rospy.Publisher('/test_pos', Marker, queue_size=1)
        self.tf_listener = tf.TransformListener()
        

        self.marker_trans = None
        self.line_set = None
        
        self.theta_thres = 0.1
        self.theta = 1.8
        self.pos_thres = 0.05
        self.points = None
        self.tf_matrix = None

    def do_getposition(self, arg):
        'get position'
        print (self.line_set[0][0][0])


    def do_quit(self, arg):
        return True
    
    def marker_callback(self, msg):
        # if self.tf_listener.canTransform(target_frame="map", source_frame="lidar_1_link", time=rospy.Time(0)):   
        #     self.tf_matrix = self.tf_listener.lookupTransform(target_frame="map", source_frame="lidar_1_link", time=rospy.Time(0))

        # res= self.tf_listener.transformPose(ps=msg.points[0], target_frame="map")
        # print(res)
        
        self.points = msg.points
        self.marker_trans = np.zeros((3,1))
        points = self.points

        num = len(points)
        thetas = np.zeros((int(num/2) -1, 1))
        
        line1 = np.zeros((2, 1))
        line2 = np.zeros((2, 1))
        
        for i in range( int(num/2) - 1):
            line1[0] = -points[2*i+1].x + points[2*i].x
            line1[1] = -points[2*i+1].y + points[2*i].y
            line2[0] = points[2*i+3].x - points[2*i+2].x
            line2[1] = points[2*i+3].y - points[2*i+2].y
            
            thetas[i] = math.acos( (line1[0]*line2[0] + line1[1] * line2[1]) / (np.linalg.norm(line1) * np.linalg.norm(line2)))
        
            if (thetas[i] < np.pi / 2.0):
                thetas[i] = np.pi - thetas[i]
            
            if (thetas[i] > self.theta - self.theta_thres):
                if (thetas[i] < self.theta + self.theta_thres):
                    pos_error = np.sqrt( pow(points[2*i+1].x - points[2*i+2].x , 2) + pow(points[2*i+1].y - points[2*i+2].y , 2))
                    
                    if (pos_error <self.pos_thres):
                        m1 = line1[1] / line1[0]
                        m2 = line2[1] / line2[0]
                        c1 = points[2*i+1].y - m1 * points[2*i+1].x
                        c2 = points[2*i+3].y - m2 * points[2*i+3].x

                        axis_vec = -line1 / np.linalg.norm(line1) - line2 / np.linalg.norm(line2)
                        axis_theta = math.atan2( axis_vec[1], axis_vec[0])

                        candidate = PoseStamped()
                        candidate.header.frame_id = "lidar_1_link"
                        candidate.pose.position.x = -(c2 -c1) / (m2 -m1)
                        candidate.pose.position.y = m1 * candidate.pose.position.x + c1
                        candidate.pose.orientation.x = 0
                        candidate.pose.orientation.y = 0
                        candidate.pose.orientation.z = np.sin(axis_theta/2)
                        candidate.pose.orientation.w = np.cos(axis_theta/2)

                        res = candidate
                        res = self.tf_listener.transformPose(ps=candidate, target_frame="map")

                        marker = Marker()
                        marker.header.frame_id = "map"
                        marker.id = 1
                        marker.type = 0
                        marker.pose = Pose()
                        marker.pose = res.pose
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        marker.color.a = 1.0
                        marker.scale.x = 0.4
                        marker.scale.y = 0.13
                        marker.scale.z = 0.13

                        self.marker_pub.publish(marker)


                    #    if (self.marker_trans[0] > 0.1):
                    #        print (line1)
                    #        print (line2)
                    #        print (thetas[i])
                           



            
        # for i in range( int(num /2 ) ) :
        #     line = np.zeros((2, 3))
        #     line[0][0] =  points[i].x
        #     line[0][1] =  points[i].y
        #     line[0][2] =  points[i].z
        #     line[1][0] =  points[i+1].x
        #     line[1][1] =  points[i+1].y
        #     line[1][2] =  points[i+1].z

        #     line_set.append(line_set)

        # print(line_set[0]) 
       
    
if __name__ == '__main__':
    ControlSuiteShell().cmdloop()
