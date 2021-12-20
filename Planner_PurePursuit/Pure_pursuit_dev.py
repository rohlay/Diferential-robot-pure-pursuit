#!/usr/bin/env python

"""
Seguimiento de trayectorias con Pure Pursuit para configuración diferencial y control PID para velocidad.

"""
#Fuentes:
#https://vinesmsuic.github.io/2020/09/29/robotics-purepersuit/#theoretical-derivation
#http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29

#Librerías de python a importar  
import numpy as np
import rospy
import tf
import sys
import geometry_msgs.msg
import matplotlib.pyplot as plt

#Saturación de velocidades (v,w) del turtlebot3
def saturacion (v, w):
	v_max, w_max = 0.22, 2.84 # [m/s] // [rad/s]: https://docplayer.es/209142691-Manual-de-usuario-del-robot-movil-turtlebot3-modelo-burger.html
	if v >= v_max:
		v = v_max
	if v <= -v_max:
		v = -v_max
	if w >= w_max:
		w = w_max
	if w <= -w_max:
		w = -w_max
	return v, w

"""Ecuaciones:
		l = np.sqrt(np.power(x,2) + np.power(y,2)) -->  La l como distancia del centro al waypoint
		x + d = r --> el radio del arco (r) y el offset en x son independientes
		r = (power(l,2))/(2*x) --> el arco del radio (r) también puede encontrarse usando l y x 
		gamma = 1/r = (2*x)/(power(l,2)) --> Curvatura
"""

"""
	Obtención del Path:
	Un path está representado por un conjunto discreto de puntos (necesita ser alamacenado en memoria)
	Se crea clase Path que contenga:

	-x location in global coordinates
	-y location in global coordinates
	-heading in global coordinates (Head Direction, may known as yaw)
	-curvature of the path at this point
	-distance (along a straight line) of this point from the beginning of the path

"""
class Path:
  def __init__(self, x, y, heading, curvature, distance): 
    self.x = x
    self.y = y
    self.heading = heading
    self.curvature = curvature
    self.distance = distance

def callback(data):
    rospy.loginfo("I heard %s",data.data)
    #Guardar datos procentes de /reference_path


if __name__ == '__main__':
	#Creación del nodo de ROS
	rospy.init_node('Pure_Pursuit')
	print("Nodo de seguimiento de trayectorias con Pure_Pursuit creado.....")

	#Creación del objeto listener
	listener = tf.TransformListener()

	#Creación del objeto publisher
	turtlebot3_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size = 1)

	#Control de la tasa de ejecución del loop
	Rate = rospy.Rate(12) #12Hz

	Path_receiver = rospy.Subscriber('/reference_path', geometry_msgs.msg.PoseStamped, callback)
	

	#Se recorre el vector de waypoints de 0 a la longitud del vector waypoints_vector
	for i in range(len(waypoints_vector)):
		#Cálculos de curvature y distance del waypoint [i] necesarios
		while not rospy.is_shutdown():
            try:
            	#This function returns two lists. The first is the (x, y, z) linear transformation of the child frame relative to the parent, and the second is the (x, y, z, w) quaternion required to rotate from the parent orientation to the child orientation.
                (trans_i,rot_i) = listener.lookupTransform('/reference_path', '/base_link', rospy.Time(0)) #Cuidado con qué waypoint esta haciendo la transformación?¿?
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


	rospy.spin()