import numpy as np
import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist
from std_msgs.msg import Float32MultiArray
import threading
import cmd, sys, os
from std_msgs.msg import Bool
import math
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus as goal_status
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import tf2_geometry_msgs
import tf2_ros

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
        # marker_sub = rospy.Subscriber('/line_markers', Marker, self.marker_callback)
        self.marker_pub = rospy.Publisher('/test_pos', Marker, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.marker_trans = None
        self.line_set = None
        
        self.theta_thres = np.deg2rad(1)
        self.theta = np.deg2rad(95)
        self.pos_thres = 0.05
        self.points = None
        self.tf_matrix = None
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.window_size = 30
        self.translation_window = []
        self.quaternion_window = []
        self.translation_filtered =[]
        self.rotation_filtered =[]
        self.rot_mat_filtered= []
        self.quaternion_filtered = []
        self.map_to_base = np.array([])
        self.preposition = False

        # self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))

    def sw_filter(self, data, window, size):
        window.append(data)
        if(len(window)>size):
            window.pop(0)
        if(len(window)<2):
            return window[0]
        ans = []
        l = len(window[0])
        for i in range(l):
            s = 0
            for vec in window:
                s += vec[i]
            ans.append(s/len(window))
        return ans
    
    def mat_from_euler(self, euler):
        alpha = euler[0]
        beta = euler[1]
        gamma = euler[2]
        sa = np.sin(alpha)		# wrt x-axis
        ca = np.cos(alpha)
        sb = np.sin(beta)		# wrt y-axis
        cb = np.cos(beta)
        sr = np.sin(gamma)		# wrt z-axis
        cr = np.cos(gamma)
        mat = [[cb*cr, sa*sb*cr - ca*sr, ca*sb*cr + sa*sr], [cb*sr, sa*sb*sr + ca*cr, ca*sb*sr - sa*cr], [-sb, sa*cb, ca*cb]]
        return mat


    def do_reach(self, arg):
        'reach to predocking position'
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = -4.162
        goal.target_pose.pose.position.y = -7.698
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = -0.7071
        goal.target_pose.pose.orientation.w = 0.7071

        self.translation_filtered = []
        self.translation_window = []
        self.quaternion_window = []
        self.client.send_goal(goal)
       

    def do_init(self, arg):
        'reach to detecting position'
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = -2.114
        goal.target_pose.pose.position.y = -5.63789
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = -1
        goal.target_pose.pose.orientation.w = 0.0

        self.translation_filtered = []
        self.translation_window = []
        self.quaternion_window = []
        self.client.send_goal(goal)

    def do_pregoal(self, arg):
        'reach to pregoal'
        calc_goal = self.calculate_pregoal()
        self.client.cancel_goal()
        if (calc_goal is False):
            print ("cannnot detect marker")
        else:
            cmd_vel = Twist()
            calc_goal = self.calculate_pregoal()
            euler = euler_from_quaternion(self.map_to_base[1])
            mat = self.mat_from_euler(euler)
            errors = [calc_goal.position.x - self.map_to_base[0][0], calc_goal.position.y - self.map_to_base[0][1]]
            x_error = -1.0 * mat[0][0] * errors[0] - 1.0 * mat[0][1] * errors[1]
            y_error = -1.0 * mat[1][0] * errors[0] - 1.0 * mat[1][1] * errors[1]
            yaw_error = self.rotation_filtered[2] - euler[2]

            while math.fabs(x_error) >= 0.01 or math.fabs(y_error) >= 0.01 or math.fabs(yaw_error) > 0.01: 
                calc_goal = self.calculate_pregoal()
                euler = euler_from_quaternion(self.map_to_base[1])
                mat = self.mat_from_euler(euler)
                errors = [calc_goal.position.x - self.map_to_base[0][0], calc_goal.position.y - self.map_to_base[0][1]]
                x_error = -1.0 * mat[0][0] * errors[0] - 1.0 * mat[0][1] * errors[1]
                y_error = -1.0 * mat[1][0] * errors[0] - 1.0 * mat[1][1] * errors[1]
                yaw_error = self.rotation_filtered[2] - euler[2]
                               
                if math.fabs(x_error) > 0.1:
                    cmd_vel.linear.x = 0.1 * math.fabs(x_error) / x_error
                else:
                    cmd_vel.linear.x = x_error

                if math.fabs(y_error) > 0.1:
                    cmd_vel.linear.y = 0.1 * math.fabs(y_error) / y_error
                else:
                    cmd_vel.linear.y = y_error
                
                if (math.fabs(yaw_error) > 0.1):
                    cmd_vel.angular.z = 0.1 * math.fabs(yaw_error) / yaw_error
                else:
                    cmd_vel.angular.z = yaw_error

                # print (cmd_vel.linear.x)
                self.vel_pub.publish(cmd_vel)

            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.angular.z = 0
            self.vel_pub.publish(cmd_vel)
    
    def do_goal(self, arg):
        'reach to pregoal'
        calc_goal = self.calculate_goal()
        if (calc_goal is False):
            print ("cannnot detect marker")
        else:
            cmd_vel = Twist()
            calc_goal = self.calculate_goal()
            euler = euler_from_quaternion(self.map_to_base[1])
            mat = self.mat_from_euler(euler)
            errors = [calc_goal.position.x - self.map_to_base[0][0], calc_goal.position.y - self.map_to_base[0][1]]
            x_error = -1.0 * mat[0][0] * errors[0] - 1.0 * mat[0][1] * errors[1]
            y_error = -1.0 * mat[1][0] * errors[0] - 1.0 * mat[1][1] * errors[1]
            yaw_error = self.rotation_filtered[2] - euler[2]
            
            while math.fabs(x_error) >= 0.01 or math.fabs(y_error) >= 0.01 or math.fabs(yaw_error) > 0.01: 
                calc_goal = self.calculate_goal()
                euler = euler_from_quaternion(self.map_to_base[1])
                mat = self.mat_from_euler(euler)
                errors = [calc_goal.position.x - self.map_to_base[0][0], calc_goal.position.y - self.map_to_base[0][1]]
                x_error = -1.0 * mat[0][0] * errors[0] - 1.0 * mat[0][1] * errors[1]
                y_error = -1.0 * mat[1][0] * errors[0] - 1.0 * mat[1][1] * errors[1]
                yaw_error = self.rotation_filtered[2] - euler[2]
                               
                if math.fabs(x_error) > 0.05:
                    cmd_vel.linear.x = 0.05 * math.fabs(x_error) / x_error
                else:
                    cmd_vel.linear.x = x_error

                if math.fabs(y_error) > 0.1:
                    cmd_vel.linear.y = 0.1 * math.fabs(y_error) / y_error
                else:
                    cmd_vel.linear.y = y_error
                
                if (math.fabs(yaw_error) > 0.1):
                    cmd_vel.angular.z = 0.1 * math.fabs(yaw_error) / yaw_error
                else:
                    cmd_vel.angular.z = yaw_error

                # print (cmd_vel.linear.x)
                self.vel_pub.publish(cmd_vel)

            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.angular.z = 0
            self.vel_pub.publish(cmd_vel)

    def calculate_pregoal(self):
        if len(self.translation_filtered) == 3:
            goal = Pose()
            euler = euler_from_quaternion([self.quaternion_filtered[0], self.quaternion_filtered[1], self.quaternion_filtered[2], self.quaternion_filtered[3]])
            mat = self.mat_from_euler(euler)
            x_offset = -0.75  # go farward is plus
            y_offset = 0.0 # left is plus

            goal.position.x = self.translation_filtered[0].item() + mat[0][0] * x_offset + mat[0][1] * y_offset 
            goal.position.y = self.translation_filtered[1].item()  + mat[1][0] * x_offset + mat[1][1] * y_offset 
            goal.position.z = 0.0
            goal.orientation.x =  self.quaternion_filtered[0]
            goal.orientation.y =  self.quaternion_filtered[1]
            goal.orientation.z =  self.quaternion_filtered[2]
            goal.orientation.w =  self.quaternion_filtered[3]

            return goal
        
        else:
            return False
        
    def calculate_goal(self):
        if len(self.translation_filtered) == 3:
            goal = Pose()
            euler = euler_from_quaternion([self.quaternion_filtered[0], self.quaternion_filtered[1], self.quaternion_filtered[2], self.quaternion_filtered[3]])
            mat = self.mat_from_euler(euler)
            x_offset = -0.6  # go farward is plus
            y_offset = 0.0 # left is plus

            goal.position.x = self.translation_filtered[0].item() + mat[0][0] * x_offset + mat[0][1] * y_offset 
            goal.position.y = self.translation_filtered[1].item()  + mat[1][0] * x_offset + mat[1][1] * y_offset 
            goal.position.z = 0.0
            goal.orientation.x =  self.quaternion_filtered[0]
            goal.orientation.y =  self.quaternion_filtered[1]
            goal.orientation.z =  self.quaternion_filtered[2]
            goal.orientation.w =  self.quaternion_filtered[3]


            return goal
        
        else:
            return False

    def do_get_data(self, arg):
        print (len(self.translation_filtered))
        self.calculate_pregoal()

    def do_quit(self, arg):
        return True
    
    def odom_callback(self, odom):
        trans_marker_in_odom = []
        self.translation = [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
        self.quaternion = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        self.odom_to_map = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time(0), rospy.Duration(1.0))
        base_in_odom = PoseStamped()
        base_in_odom.header.frame_id = 'odom'
        base_in_odom.header.stamp = odom.header.stamp
        base_in_odom.pose = odom.pose.pose
        base_in_map = tf2_geometry_msgs.do_transform_pose(base_in_odom, self.odom_to_map)
        self.base_in_map_translation = [base_in_map.pose.position.x, base_in_map.pose.position.y, base_in_map.pose.position.z]		
        self.base_in_map_quaternion = [base_in_map.pose.orientation.x, base_in_map.pose.orientation.y, base_in_map.pose.orientation.z, base_in_map.pose.orientation.w]
        euler = euler_from_quaternion(self.quaternion)
        mat = self.mat_from_euler(euler)
        self.base_to_odom = np.array(mat)
        print (self.base_to_odom)
		
        
    
    def marker_callback(self, msg):
        # if self.tf_listener.canTransform(target_frame="map", source_frame="lidar_2_link", time=rospy.Time(0)):   
        #     self.tf_matrix = self.tf_listener.lookupTransform(target_frame="map", source_frame="lidar_2_link", time=rospy.Time(0))

        # res= self.tf_listener.transformPose(ps=msg.points[0], target_frame="map")
        # print(res)
 
        self.map_to_base = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0), ros::Duration(1))
        
        
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
            print (thetas)
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
                        candidate.header.frame_id = "lidar_2_link"
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
                        trans_marker_in_map =[res.pose.position.x, res.pose.position.y, 0]
                        quat_marker_in_map = [0, 0, res.pose.orientation.z, res.pose.orientation.w]
                        self.translation_filtered = self.sw_filter(trans_marker_in_map, self.translation_window, self.window_size)
                        self.quaternion_filtered = self.sw_filter(quat_marker_in_map, self.quaternion_window, self.window_size)
                        self.rotation_filtered = euler_from_quaternion(self.quaternion_filtered)
                        self.rot_mat_filtered = self.mat_from_euler(self.rotation_filtered)
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
