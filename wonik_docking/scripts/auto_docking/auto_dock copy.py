#!/usr/bin/env python3


import time
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from wonik_docking.srv import auto_docking
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus as goal_status
import actionlib
from nav_msgs.msg import Odometry
import tf2_geometry_msgs
import tf2_ros
import tf
from visualization_msgs.msg import Marker

class Docking():
	# initialization
	def __init__(self):
		# node, server, publishers, subscribers
		self.node_name = 'auto_docking'
		rospy.init_node(self.node_name)
		frequency = 10
		self.rate = rospy.Rate(frequency)
		rospy.set_param('docking', False)
		
		self.docking_server = rospy.Service(self.node_name, auto_docking, self.service_callback)
		
		self.mkr_pub = rospy.Publisher('/docking_marker', Marker, queue_size=1)
		self.vel_pub = rospy.Publisher('/cmd_vel_dock', Twist, queue_size=1)
		self.pose_sub = rospy.Subscriber('/line_markers', Marker, self.line_callback)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
		self.tf_listener = tf.TransformListener()
		self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

		self.map_to_base = []
		
		# ROS PARAM
		theta_thres = rospy.get_param('/'+self.node_name+'/theta_theshold')
		self.theta_thres = np.deg2rad(theta_thres)
		theta_des = rospy.get_param('/'+self.node_name+'/marker/shape')
		self.theta_des = np.deg2rad(theta_des)
		self.pos_thres = rospy.get_param('/'+self.node_name+'/position_theshold')
		self.window_size = rospy.get_param('/'+self.node_name+'/windows_size')
		self.backward = rospy.get_param('/'+self.node_name+'/docking_backward')
		self.offset = [rospy.get_param('/'+self.node_name+'/marker'+'/offset/x'), rospy.get_param('/'+self.node_name+'/marker'+'/offset/y'), np.deg2rad(rospy.get_param('/'+self.node_name+'/marker'+'/offset/theta'))]
		
		self.reach_goal = MoveBaseGoal()
		self.reach_goal.target_pose.header.frame_id = 'map'
		self.reach_goal.target_pose.pose.position.x = rospy.get_param('/'+self.node_name+'/reach/pos/x')
		self.reach_goal.target_pose.pose.position.y = rospy.get_param('/'+self.node_name+'/reach/pos/y')
		self.reach_goal.target_pose.pose.position.z = rospy.get_param('/'+self.node_name+'/reach/pos/z')
		self.reach_goal.target_pose.pose.orientation.x = rospy.get_param('/'+self.node_name+'/reach/quat/x')
		self.reach_goal.target_pose.pose.orientation.y = rospy.get_param('/'+self.node_name+'/reach/quat/y')
		self.reach_goal.target_pose.pose.orientation.z = rospy.get_param('/'+self.node_name+'/reach/quat/z')
		self.reach_goal.target_pose.pose.orientation.w = rospy.get_param('/'+self.node_name+'/reach/quat/w')

		self.precontact = rospy.get_param('/' +self.node_name+'/precontact_position')

		self.translation_window = []
		self.quaternion_window = []
		self.docking_stage = 0


	# establish the rotation matrix from euler angle
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
		# euler rotation matrix
		mat = [[cb*cr, sa*sb*cr - ca*sr, ca*sb*cr + sa*sr], [cb*sr, sa*sb*sr + ca*cr, ca*sb*sr - sa*cr], [-sb, sa*cb, ca*cb]]
		return mat

	def pure_pursuit(self):
		v = np.zeros((3,1))

		start_vel_x = self.odom.twist.twist.linear.x
		print(start_vel_x)

		return v

	def odom_callback(self, msg):
		self.odom = msg


	# callback of line_detection
	def line_callback(self, msg):
		self.map_to_base = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
		self.points = msg.points
		self.marker_trans = np.zeros((3,1))
		points = self.points

		num = len(points)
		self.line_checker = False

		if (num > 2):
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
		
		for i in range( int(num/2) - 1):    
			if (math.fabs(thetas[i] - self.theta_des) <= self.theta_thres):
				pos_error = np.sqrt( pow(points[2*i+1].x - points[2*i+2].x , 2) + pow(points[2*i+1].y - points[2*i+2].y , 2))
				line1[0] = -points[2*i+1].x + points[2*i].x
				line1[1] = -points[2*i+1].y + points[2*i].y
				line2[0] = points[2*i+3].x - points[2*i+2].x
				line2[1] = points[2*i+3].y - points[2*i+2].y
				if (i < int(num/2) - 1):
					if (pos_error <self.pos_thres and math.fabs(thetas[i+1] - np.deg2rad(90) - self.theta_des / 2.0) < self.theta_thres ):
						m1 = line1[1] / line1[0]
						m2 = line2[1] / line2[0]
						c1 = points[2*i+1].y - m1 * points[2*i+1].x
						c2 = points[2*i+3].y - m2 * points[2*i+3].x

						axis_vec = -line1 / np.linalg.norm(line1) - line2 / np.linalg.norm(line2)
						axis_theta = math.atan2( axis_vec[1], axis_vec[0])

						marker = Marker()
						marker.header.frame_id = "map"
						marker.id = 1
						marker.type = 0
						marker.pose = Pose()
						marker.pose.position.x = -(c2 -c1) / (m2 -m1)
						marker.pose.position.y = m1 * marker.pose.position.x + c1
						marker.pose.orientation.x = 0
						marker.pose.orientation.y = 0
						marker.pose.orientation.z = np.sin(axis_theta/2)
						marker.pose.orientation.w = np.cos(axis_theta/2)
						
						marker.color.r = 0.0
						marker.color.g = 1.0
						marker.color.b = 0.0
						marker.color.a = 1.0
						marker.scale.x = 0.4
						marker.scale.y = 0.13
						marker.scale.z = 0.13

						self.mkr_pub.publish(marker)
						self.line_checker = True
						self.do_filtered([marker.pose.position.x, marker.pose.position.y, 0], [0, 0, marker.pose.orientation.z, marker.pose.orientation.w]) 

	# filtering with moving windows
	def do_filtered(self, trans_marker_in_map, quat_marker_in_map):
		self.translation_filtered = self.sw_filter(trans_marker_in_map, self.translation_window, self.window_size)
		self.quaternion_filtered = self.sw_filter(quat_marker_in_map, self.quaternion_window, self.window_size)
		self.rotation_filtered = euler_from_quaternion(self.quaternion_filtered)
		self.rot_mat_filtered = self.mat_from_euler(self.rotation_filtered)

	# sliding window
	def sw_filter(self, data, window, size):
		window.append(data)
		if(len(window)>size):
			window.pop(0)
		# calculate the average
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

	# check for limits of velocity
	def limit_check(self, vel, uper, lower):
		if(uper and abs(vel)>uper):
			vel = np.sign(vel) * uper
		elif(lower and abs(vel)<lower):
			vel = np.sign(vel) * lower
		return vel
	
	def calculate_goal(self, stage):
		goal = Pose()
		euler = euler_from_quaternion([self.quaternion_filtered[0], self.quaternion_filtered[1], self.quaternion_filtered[2], self.quaternion_filtered[3]])
		mat = self.mat_from_euler(euler)
            
		if (stage == 1):
			x_offset = self.offset[0] - self.precontact
			y_offset = self.offset[1]

			goal.position.x = self.translation_filtered[0].item() + mat[0][0] * x_offset + mat[0][1] * y_offset 
			goal.position.y = self.translation_filtered[1].item()  + mat[1][0] * x_offset + mat[1][1] * y_offset 
			goal.position.z = 0.0
			goal.orientation.x = self.quaternion_filtered[0]
			goal.orientation.y = self.quaternion_filtered[1]
			if (self.backward):
				goal.orientation.z = -self.quaternion_filtered[2]
			else:
				goal.orientation.z = self.quaternion_filtered[2]
			goal.orientation.w = self.quaternion_filtered[3]

			return goal
		
		elif (stage == 2):
			x_offset = self.offset[0]
			y_offset = self.offset[1]

			goal.position.x = self.translation_filtered[0].item() + mat[0][0] * x_offset + mat[0][1] * y_offset 
			goal.position.y = self.translation_filtered[1].item()  + mat[1][0] * x_offset + mat[1][1] * y_offset 
			goal.position.z = 0.0
			goal.orientation.x = self.quaternion_filtered[0]
			goal.orientation.y = self.quaternion_filtered[1]
			if (self.backward):
				goal.orientation.z = -self.quaternion_filtered[2]
			else:
				goal.orientation.z = self.quaternion_filtered[2]
			goal.orientation.w = self.quaternion_filtered[3]

			return goal

	def do_stage0(self):
		self.client.send_goal(self.reach_goal)

	def do_stage1(self):
		v = self.pure_pursuit()

		stime = rospy.Time.now()
		line_checker = False
		self.client.cancel_goal()
		while (stime + rospy.Duration(15.0) > rospy.Time.now()):
			if (self.line_checker):
				line_checker = True
				break
		
		if (not line_checker):
			rospy.loginfo("Robot cannot detect marker. Services is rejected.")
			return False

		cmd_vel = Twist()
		calc_goal = self.calculate_goal(1)
		euler = euler_from_quaternion(self.map_to_base[1])
		mat = self.mat_from_euler(euler)
		errors = [calc_goal.position.x - self.map_to_base[0][0], calc_goal.position.y - self.map_to_base[0][1]]
		print (errors[0], errors[1])
		x_error = errors[0]
		y_error = errors[1]
		yaw_error = self.rotation_filtered[2] - euler[2]

		if self.backward:
			if (yaw_error < -1.57):
				yaw_error = yaw_error + 3.141592
			elif (yaw_error > 1.57):
				yaw_error = yaw_error - 3.141592
		
				
		if (math.fabs(yaw_error) > 6.2830):
			yaw_error = yaw_error - yaw_error/math.fabs(yaw_error) * 6.2830

		rospy.loginfo("Go to Precontact Position.")
		stime = rospy.Time.now()
		while math.fabs(x_error) >= 0.01 or math.fabs(y_error) >= 0.01 or math.fabs(yaw_error) > 0.01: 
			calc_goal = self.calculate_goal(1)
			euler = euler_from_quaternion(self.map_to_base[1])
			mat = self.mat_from_euler(euler)
			errors = [calc_goal.position.x - self.map_to_base[0][0], calc_goal.position.y - self.map_to_base[0][1]]
			x_error = errors[0]
			y_error = errors[1]
			yaw_error = self.rotation_filtered[2] - euler[2]
			
			if self.backward:
				if (yaw_error < -1.57):
					yaw_error = yaw_error + 3.141592
				elif (yaw_error > 1.57):
					yaw_error = yaw_error - 3.141592

			if math.fabs(x_error) > 0.1:
				cmd_vel.linear.x = 0.1 * math.fabs(x_error) / x_error
			else:
				cmd_vel.linear.x = x_error

			if math.fabs(y_error) > 0.1:
				cmd_vel.linear.y = 0.1 * math.fabs(y_error) / y_error
			else:
				cmd_vel.linear.y = y_error
			
			if (math.fabs(yaw_error) > 0.15):
				cmd_vel.angular.z = 0.15 * math.fabs(yaw_error) / yaw_error
			else:
				cmd_vel.angular.z = yaw_error

			self.vel_pub.publish(cmd_vel)

			if (stime + rospy.Duration(10.0) < rospy.Time.now()):
				break

		cmd_vel.linear.x = 0
		cmd_vel.linear.y = 0
		cmd_vel.angular.z = 0
		self.vel_pub.publish(cmd_vel)

		return True
	
	def do_stage2(self):
		self.client.cancel_goal()
		stime = rospy.Time.now()
		line_checker = False
		while (stime + rospy.Duration(15.0) > rospy.Time.now()):
			if (self.line_checker):
				line_checker = True
				break
		
		if (not line_checker):
			rospy.loginfo("Robot cannot detect marker. Services is rejected.")
			return False

		cmd_vel = Twist()
		calc_goal = self.calculate_goal(2)
		euler = euler_from_quaternion(self.map_to_base[1])
		mat = self.mat_from_euler(euler)
		errors = [calc_goal.position.x - self.map_to_base[0][0], calc_goal.position.y - self.map_to_base[0][1]]
		x_error = errors[0]
		y_error = errors[1]
		yaw_error = self.rotation_filtered[2] - euler[2]

		if self.backward:
			if (yaw_error < -1.57):
				yaw_error = yaw_error + 3.141592
			elif (yaw_error > 1.57):
				yaw_error = yaw_error - 3.141592
		
		if (math.fabs(yaw_error) > 6.2830):
			yaw_error = yaw_error - yaw_error/math.fabs(yaw_error) * 6.2830

		rospy.loginfo("Go to Docking Position.")
		stime = rospy.Time.now()
		while math.fabs(x_error) >= 0.015 or math.fabs(y_error) >= 0.01 or math.fabs(yaw_error) > 0.01: 
			calc_goal = self.calculate_goal(2)
			euler = euler_from_quaternion(self.map_to_base[1])
			mat = self.mat_from_euler(euler)
			errors = [calc_goal.position.x - self.map_to_base[0][0], calc_goal.position.y - self.map_to_base[0][1]]
			x_error = errors[0]
			y_error = errors[1]
			yaw_error = self.rotation_filtered[2] - euler[2]
			
			if self.backward:
				if (yaw_error < -1.57):
					yaw_error = yaw_error + 3.141592
				elif (yaw_error > 1.57):
					yaw_error = yaw_error - 3.141592
			

			if (math.fabs(yaw_error) > 6.2830):
				yaw_error = yaw_error - yaw_error/math.fabs(yaw_error) * 6.2830

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

			self.vel_pub.publish(cmd_vel)

			if (stime + rospy.Duration(10.0) < rospy.Time.now()):
				break

		cmd_vel.linear.x = 0
		cmd_vel.linear.y = 0
		cmd_vel.angular.z = 0
		self.vel_pub.publish(cmd_vel)

		return True
	
	def do_undocking(self):
		stime = rospy.Time.now()
		while (stime + rospy.Duration(5.0) > rospy.Time.now()):
			cmd_vel = Twist()
			if (self.backward):
				cmd_vel.linear.x = 0.1
			else:
				cmd_vel.linear.x = -0.1

			self.vel_pub.publish(cmd_vel)

		cmd_vel.linear.x = 0
		cmd_vel.linear.y = 0
		cmd_vel.angular.z = 0
		self.vel_pub.publish(cmd_vel)

	# callback function of service /auto_docking
	def service_callback(self, auto_docking):
		self.client.cancel_goal()
		docking_state = rospy.get_param('docking')

		if (auto_docking.individual):
			rospy.set_param('docking', False)
			if (auto_docking.stage == 0):
				if self.docking_stage == 0:
					self.do_stage0()
					self.docking_stage == 1
				else:
					rospy.loginfo("Docking step is wrong.")
				
			elif  (auto_docking.stage == 1):
				if self.docking_stage == 1 or self.docking_stage == 0 :
					self.docking_stage = 2
					self.do_stage1()
				else:
					rospy.loginfo("Docking step is wrong.")
			else:
				if self.docking_stage == 2:
					self.do_stage2()
					rospy.set_param('docking', True)

			return "Service request received"
		
		if (not auto_docking.docking):
			rospy.set_param('docking', False)
			if (self.docking_stage == 2):
				self.do_undocking()
				self.docking_stage = 0
			else:
				rospy.loginfo("Robot isn't docked. Services is rejected.")
			return "Service request received"
		
		else:
			if(not docking_state or auto_docking.stage == 0): 
				rospy.set_param('docking', True)
				rospy.loginfo("Service request received.")
				self.docking_stage = 0

				self.do_stage0()
				if(self.client.wait_for_result()):
					res = self.do_stage1()
			
					if (res):
						self.docking_stage = 1
						res2 = self.do_stage2()
						if (res2):
							self.docking_stage = 2
							rospy.loginfo("docking is done")
						else:
							rospy.loginfo("docking isfailed")
					else:
						rospy.loginfo("pre docking is failed")
			else:
				if (auto_docking.stage == 1):
					rospy.loginfo("Service request received.")
					if (res):
						self.docking_stage = 1
						res2 = self.do_stage2()
						if (res2):
							self.docking_stage = 2
							rospy.loginfo("docking is done")
						else:
							rospy.loginfo("docking isfailed")
					else:
						rospy.loginfo("pre docking is failed")

				elif (auto_docking.stage == 2):					
					rospy.loginfo("Service request received.")
					res2 = self.do_stage2()
					if (res2):
						self.docking_stage = 2
						rospy.loginfo("docking is done")
					else:
						rospy.loginfo("docking isfailed")

			return "Service request received"

if __name__ == '__main__':
	my_docker = Docking()
	while(not rospy.is_shutdown()):

		my_docker.rate.sleep()
