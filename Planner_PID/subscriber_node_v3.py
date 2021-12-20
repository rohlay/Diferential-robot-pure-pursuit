#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

#Guardar datos procentes de /planner_ros_node/stamped_array
class Nodo(object):
	def __init__(self):
			# Parameters
		# Positions:
			# Waypoints positions
	       	self.waypoints_pos_x = []
	       	self.waypoints_pos_y = []
	       	self.waypoints_pos_z = []

	       	# Odom positions
	       	self.odom_pos_x = []
	       	self.odom_pos_y = []
	       	self.odom_pos_z = []

	    # Orientations:
	       	# Waypoints quaternion
	       	self.waypoints_quat_x = []
	       	self.waypoints_quat_y = []
	       	self.waypoints_quat_z = []
	       	self.waypoints_quat_w = []

	       	# Odom quaternion
	       	self.odom_quat_x = []
	       	self.odom_quat_y = []
	       	self.odom_quat_z = []
	       	self.odom_quat_w = []

	        # Node cycle rate (in Hz).
	        self.loop_rate = rospy.Rate(10)

	        # Publishers
	        rospy.Subscriber('/planner_ros_node/stamped_array', PoseStamped, self.callback_waypoints)
	        rospy.Subscriber('/odom', Odometry, self.callback_odom)

	def callback_waypoints(self, data):
			self.waypoints_pos_x.append(data.pose.position.x)		#Obtención de las componentes de los waypoints en vectores con longitud variable según el número de waypoints	
			self.waypoints_pos_y.append(data.pose.position.y)
			self.waypoints_pos_z.append(data.pose.position.z)
			self.waypoints_quat_x.append(data.pose.orientation.x)
			self.waypoints_quat_y.append(data.pose.orientation.y)
			self.waypoints_quat_z.append(data.pose.orientation.z)
			self.waypoints_quat_w.append(data.pose.orientation.w)

	def callback_odom(self, data):
			self.odom_pos_x = data.pose.pose.position.x		#Obtención en tiempo real de las componentes de la odometría (se van actualizando en cada loop)   	
			self.odom_pos_y = data.pose.pose.position.y
			self.odom_pos_z = (data.pose.pose.position.z)
			self.odom_quat_x = (data.pose.pose.orientation.x)
			self.odom_quat_y = (data.pose.pose.orientation.y)
			self.odom_quat_z = (data.pose.pose.orientation.z)
			self.odom_quat_w = (data.pose.pose.orientation.w)

	def start(self):
	        rospy.loginfo("Starting Node...")

	        while not rospy.is_shutdown():
	                print(mi_nodo.waypoints_pos_x)

	                 #Parte de cómputo del PurePursuit:









	                self.loop_rate.sleep()	

if __name__ == '__main__':
	rospy.init_node('subscriber_node')
	mi_nodo = Nodo()
	mi_nodo.start()

