#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def callback(waypoints):
    print waypoints
    #Guardar datos procentes de /reference_path

Path_receiver = rospy.Subscriber('/reference_path', PoseStamped, waypoints)

rospy.spin()