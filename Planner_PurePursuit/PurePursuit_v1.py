#Código basado en: https://www.youtube.com/watch?v=b348R-G8wO0&ab_channel=RoboTechs

#!/usr/bin/env python

"""
Seguimiento de trayectorias con Pure Pursuit para configuración diferencial y control PID para velocidad.

"""

#Librerías de python a importar  
import numpy as np
import rospy
import tf
import sys
import geometry_msgs.msg
import pickle
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


#Función main del nodo
if __name__ == '__main__':
	#Creación del nodo de ROS
	rospy.init_node('Pure_Pursuit')
	print("Nodo de seguimiento de trayectorias con Pure_Pursuit creado.....")

	# Parámetros de control
	k1, k2 = 1.0, 5.0

	#Creación del objeto listener
	listener = tf.TransformListener()

	#Creación del objeto publisher
	turtlebot3_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size = 1)

	#Creación de objeto Twist para las velocidades calculadas
	vel_calcu = geometry_msgs.msg.Twist()

	#Control de la tasa de ejecución del loop
	frecuencia = rospy.Rate(12) #12Hz

	#Obtención de la trayectoria:
	```
	¿Como obtener la trayectoria, se guarda en un vector? ¿cómo se recibe?

	```

	#vd array de poseStamped¿?¿?¿
	#Para todos los puntos de la trayectoria:
	for i in range(len(vd)):
		got_odom_data = False 		#Se resetea el flag para volver al bucle de try-except
		while not got_odom_data:
			try:
				translacion, rotacion = listener.lookupTransform('/odom','/base_link',rospy.Time(0))
				got_odom_data = True
			except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

		# Se obtiene ángulos de euler a traves del quaternion:	https://www.programcreek.com/python/example/103985/tf.transformations.euler_from_quaternion
		eul_rot = tf.transformations.euler_from_quaternion(rotacion)

		#Se define el punto de la trayectoria como: (x,y,yaw)
		p_tray = [translacion[0],translacion[1],eul_rot[2]]

		#Error respecto del Pose actual para cada waypoint
		err_local = [trans[0] - path_data[i][0], trans[1] - path_data[i][1], eul_rot[2] - path_data[i][2]]
		err_global = [np.cos(path_data[i][2])*err_local[1]+np.sin(path_data[i][2])*err_local[2], -np.sin(path_data[i][2]*err_local[1]+np.cos(path_data[i][2]*err_local[1]))]

		#Computar la entrada del control
		v_in = k1.np.sqrt(err_local[0]**2)

		#................
		#sigue