#!/usr/bin/env python
# coding=utf-8

import math
import rospy
import rospy
from geometry_msgs.msg  import Twist
from std_msgs.msg       import Float64
import numpy 
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32
from std_msgs.msg import Float32

#Guardar datos procentes de /planner_ros_node/stamped_array
class Nodo(object):
	def __init__(self):
			
		# WAYPOINTS GLOBAL (recibimos del planificador)
			# Waypoints que recibimos y almacenamos
		self.xp_wp = []
		self.yp_wp = [] 
		self.zp_wp = [] 
		self.xo_wp = [] 
		self.yo_wp = [] 
		self.zo_wp = [] 
		self.wo_wp = []   

	
		#WAYPOINTS LOCAL (envíamos al controlador) 
		
		
		self.xp = [0.0]
		self.yp = [0.0] 

		"""
		self.wpt_msg = PoseStamped()
		self.wpt_msg.xp = []
			self.wpt_msg.yp = [] 
			self.wpt_msg.zp = [] 
			self.wpt_msg.xo = [] 
			self.wpt_msg.yo = [] 
			self.wpt_msg.zo = [] 
			self.wpt_msg.wo = [] 
		"""

			# Odom
		self.odom_pos_x = 0.0
		self.odom_pos_y = 0.0
		self.odom_pos_z = 0.0
		self.odom_quat_x = 0.0
		self.odom_quat_y = 0.0
		self.odom_quat_z = 0.0
		self.odom_quat_w = 0.0
			
		# flag que se activa cuando el nodo recibe waypoints
		self.flag = 0
		self.interp_y = 0
	
			# Distancia al waypoint por defecto
		self.distance = 100
		self.distance_anterior = 0

		self.callback_counter = 0

		#contador para feeding of waypoints:
		self.cont_wp = 0

		#Variable que recibe del planner para saber la longitud del vector de waypoints
		self.length_waypoints = 0

		# Node cycle rate (in Hz).
		self.loop_rate = rospy.Rate(20)

		# Subscribers
		rospy.Subscriber('/planner_ros_node/stamped_array', PoseStamped, self.callback_waypoints)
		rospy.Subscriber('/diff_drive_go_to_goal/distance_to_goal', Float32, self.callback_distance)
		rospy.Subscriber('/odom', Odometry, self.callback_odom)
		rospy.Subscriber('/planner_ros_node/length_waypoints',UInt32, self.callback_length)

			#Publishers
		#Se publican las distancias al goal de manera que se pueda subscribir el planner_ros_node e ir entregando waypoints de 1 en 1 según valor de d
		self.d_pub = rospy.Publisher('~distance_to_goal_v2', Float32, queue_size=10)            
		self.waypoints_pub = rospy.Publisher('~waypoints_ordenados', PoseStamped,  queue_size=10)

		self.waypoints_xy_pub = rospy.Publisher('~waypoints_xy', Float64, queue_size=50)
		
		self.goalMsg = PoseStamped()




	def callback_waypoints(self, data):				# Obtención de las componentes de los waypoints en vectores con longitud variable según el número de waypoints
								# callback se activa cuando un mensage se envía por el nodo, cuando se activa almacenamos los waypoints en un vector
		self.xp_wp.append(data.pose.position.x)         
		self.yp_wp.append(data.pose.position.y)
		self.zp_wp.append(data.pose.position.z)
		self.xo_wp.append(data.pose.orientation.x)
		self.yo_wp.append(data.pose.orientation.y)
		self.zo_wp.append(data.pose.orientation.z)
		self.wo_wp.append(data.pose.orientation.w)


		self.callback_counter += 1
		print(self.callback_counter)
		#print(len(self.xp_wp))
		"""
		if len(self.xp_wp) == mi_nodo.length_waypoints: 
			mi_nodo.flag=1
		"""

	def callback_odom(self, data):				#Obtención en tiempo real de las componentes de la odometría (se van actualizando en cada loop) 
		self.odom_pos_x = data.pose.pose.position.x         
		self.odom_pos_y = data.pose.pose.position.y
		self.odom_pos_z = data.pose.pose.position.z
		self.odom_quat_x = data.pose.pose.orientation.x
		self.odom_quat_y = data.pose.pose.orientation.y
		self.odom_quat_z = data.pose.pose.orientation.z
		self.odom_quat_w = data.pose.pose.orientation.w

	def callback_distance(self, data):
		self.distance = data.data

	def callback_length(self, data):
		self.length_waypoints = data.data

	def start(self):

		rospy.loginfo("Iniciando nodo..")

		while not rospy.is_shutdown():
					
					   
						#mi_nodo.d_pub.publish(mi_nodo.distance)
						#mi_nodo.waypoints_pub.publish(mi_nodo.goalMsg)
					   
			

			"""
			for mi_nodo.cont_wp in n_waypoints:	# Almacenar en un vector los waypoints que recibimos uno por uno
				
				mi_nodo.goalMsg = PoseStamped()
				mi_nodo.goalMsg.header.frame_id = "base_link"
				mi_nodo.goalMsg.header.stamp = rospy.Time(0)
				mi_nodo.goalMsg.pose.position.x = mi_nodo.xp_wp[mi_nodo.cont_wp]
				mi_nodo.goalMsg.pose.position.y = mi_nodo.yp_wp[mi_nodo.cont_wp]
				mi_nodo.goalMsg.pose.position.z = mi_nodo.zp_wp[mi_nodo.cont_wp]
				mi_nodo.goalMsg.pose.orientation.x = mi_nodo.xo_wp[mi_nodo.cont_wp]
				mi_nodo.goalMsg.pose.orientation.y = mi_nodo.yo_wp[mi_nodo.cont_wp]
				mi_nodo.goalMsg.pose.orientation.z = mi_nodo.zo_wp[mi_nodo.cont_wp]
				mi_nodo.goalMsg.pose.orientation.w = mi_nodo.wo_wp[mi_nodo.cont_wp]

				print(mi_nodo.goalMsg)
				mi_nodo.waypoints_pub.publish(mi_nodo.goalMsg)

			"""
			#rospy.sleep(5)
			#print(len(mi_nodo.xp_wp))
			#print(mi_nodo.length_waypoints)
			#print(range(len(mi_nodo.xp_wp)))

			#print(mi_nodo.callback_counter)

			if (len(mi_nodo.xp_wp) == mi_nodo.length_waypoints) and (len(mi_nodo.xp_wp) > 0): 
				mi_nodo.flag=1
			
			#rospy.sleep(2)  
					
			# INTERPOLADOR
			if mi_nodo.flag == 1:
				
				#"""
				# interpolacion con 2 puntos ==> rectas
				for i in range(len(mi_nodo.xp_wp)-1):	# recorre el vector de waypoints 0 a n-1
					print("waypoint: ")
					print(i)
					
					xmin = mi_nodo.xp_wp[i]
					xmax = mi_nodo.xp_wp[i+1]

					if(xmin==xmax):
						mi_nodo.interp_y = 1;

					ymin = mi_nodo.yp_wp[i]
					ymax = mi_nodo.yp_wp[i+1]

					if(ymin==ymax):
						mi_nodo.interp_y = 0;
						
					print("Interpola en y:")
					print(mi_nodo.interp_y)
					print(mi_nodo.xp_wp[i])
					print(mi_nodo.yp_wp[i])	
		
								
					
					# número de puntos en función de la distancia entre dos waypoints  - para linspace
					# si están cerca, menos puntos
					# si están lejos, más puntos
					distancia = math.sqrt((xmax-xmin)**2 + (ymax-ymin)**2)
					npoints = round(distancia * 60) 

					
					
					if mi_nodo.interp_y == 0:	# interpolación (unidimensional) sobre la variable x
						y = [ymin, ymax]
						if xmin < xmax: # si x creciente
							x = [xmin, xmax]
							
							f = CubicSpline(x,y, bc_type = 'natural')	
							x_new = numpy.linspace(xmin, xmax, npoints)
							y_new = f(x_new)
								
						if xmin > xmax: # si x decreciente
							x = [xmax, xmin]
							
							f = CubicSpline(x,y, bc_type = 'natural')	
							x_new = numpy.linspace(xmin, xmax, npoints)
							y_new = f(x_new)
						
							# si x decreciente, damos la vuelta al vector
							#x_new[:] = x_new[::-1]
							y_new[:] = y_new[::-1]
													
							
					if mi_nodo.interp_y == 1:
						x = [xmin, xmax]
						if ymin < ymax: # si y creciente
							y = [ymin, ymax]
							
							f = CubicSpline(y,x, bc_type = 'natural')	
							y_new = numpy.linspace(ymin, ymax, npoints)
							x_new = f(y_new)
								
						if ymin > ymax: # si y decreciente
							y = [ymax, ymin]
							
							f = CubicSpline(y,x, bc_type = 'natural')	
							y_new = numpy.linspace(ymin, ymax, npoints)
							x_new = f(y_new)
						
							# si x decreciente, damos la vuelta al vector
							#x_new[:] = x_new[::-1]
							y_new[:] = y_new[::-1]
					
					for j in range(int(npoints)):
						mi_nodo.xp.append(x_new[j])
						mi_nodo.yp.append(y_new[j])

						#TODO Calcular quaternion y la Z y luego meterselo en mensajes PoseStamped
						#mi_nodo.wpt_msg.zp 
						#mi_nodo.wpt_msg.xo 
						#mi_nodo.wpt_msg.yo  
						#mi_nodo.wpt_msg.zo  
						#mi_nodo.wpt_msg.wo  
								
					
		
					print("distancia: ")
					print(distancia)
					print("npoints: ")
					print(npoints)
					print("\n")


				print(mi_nodo.xp)
				print(mi_nodo.yp)

				for l in range(len(mi_nodo.xp)):
					mi_nodo.waypoints_xy_pub.publish(mi_nodo.xp[l]) 
					mi_nodo.waypoints_xy_pub.publish(mi_nodo.yp[l])
					
				plt.figure(figsize = (10,8))
				plt.plot(mi_nodo.xp, mi_nodo.yp, 'b')
				x = mi_nodo.xp_wp
				y = mi_nodo.yp_wp
				plt.plot(x, y, 'ro')
				plt.title('Cubic Spline Interpolation')
				plt.xlabel('x')
				plt.ylabel('y')
				plt.show()



				npoints = 0
				mi_nodo.interp_y = 0
				mi_nodo.flag = 0

			#mi_nodo.waypoints_pub.publish(mi_nodo.wpt_msg)
			

		self.loop_rate.sleep()  

if __name__ == '__main__':
	rospy.init_node('interpolador')
	mi_nodo = Nodo()
	mi_nodo.start()

