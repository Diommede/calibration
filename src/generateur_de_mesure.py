#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math as math	
from std_msgs.msg import String
from rospy import init_node, is_shutdown
from parametres import Definitions

####################################
###########PUBLIEUR ROS#############
####################################

def talker(info):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "%s" % info
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

####################################
############MAIN FUNCTION###########
####################################

#def main():
def generateur():
	Donnees = Definitions()

####################################
##############CALCULS###############
####################################	


	for i in range(0,8):  
		Donnees.l[i] = (-Donnees.dco0 - np.matmul(np.transpose(Donnees.Nw), (np.matmul(Donnees.Rwb[i],Donnees.tbc) + Donnees.twb[i]))) / (np.matmul(np.matmul(np.matmul(np.transpose(Donnees.Nw), Donnees.Rwb[i]),Donnees.Rbc),Donnees.e3))

	return Donnees.l

####################################
##############DEBUG#################
####################################


#if __name__ == "__main__":
#    main()
