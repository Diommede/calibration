#!/usr/bin/python
# -*- coding: utf-8 -*-

import rosbag
import rospy
import numpy as np
import math as math	
from geometry_msgs.msg import PoseStamped
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32MultiArray
from rospy import init_node, is_shutdown
from generateur_de_mesure import generateur
from parametres import Definitions
from tf import transformations as tf


####################################
############MAIN FUNCTION###########
####################################

class main:

####################################
##############CALCULS###############
####################################	
	def __init__(self, Filename):

#Lire les données du bag

		Donnees = Definitions()

		for i in range(1,7):
			print(np.array(Donnees.m).shape)
			np.append(Donnees.m, np.concatenate((np.matmul(np.transpose(Donnees.Nw),Donnees.RwbDeg[i])*Donnees.l[i],np.matmul(Donnees.Nw,(Donnees.RwbDeg[i]-Donnees.I)),[-1])))
			np.append(Donnees.b, np.dot(Donnees.Nw,Donnees.twb[i]))
	
		#Donnees.x = np.linalg.lstsq(Donnees.m, -Donnees.b)

######################################
################ROS###################
######################################

#Initiation du noeud , subscriber et publisher
	  	rospy.init_node('autocalib', anonymous=True)
		rospy.Subscriber('sub', String, self.callback)
		self.pub = rospy.Publisher('chatter', String, queue_size=10)

#Crée un bag en mode écriture
   		bag = rosbag.Bag('test.bag', 'w')

#Ecriture du bag pour test

		mot =  PoseStamped()
		mot2 = PoseStamped()

		mot.pose.position.x = 54.2
		mot.pose.position.y = 50.1
		mot.pose.position.z = 59.0

		mot.pose.orientation.x = 44.2
		mot.pose.orientation.y = 40.1
		mot.pose.orientation.z = 49.0
		mot.pose.orientation.w = 48.7


		mot2.pose.position.x = 34.2
		mot2.pose.position.y = 30.1
		mot2.pose.position.z = 39.0

		mot2.pose.orientation.x = 24.2
		mot2.pose.orientation.y = 20.1
		mot2.pose.orientation.z = 29.0
		mot2.pose.orientation.w = 28.7

                bag.write('/dist', mot2)
		bag.write('/pose', mot)




		with open("/home/denis/catkin_ws/src/autocalibration/src/data.txt", "r") as f:
    			for line in f.readlines():
        			if 'Nw' in line:					
					a,b = line.split(':')
					Donnees.Nw = b
					print (b)

		bag.close()
   		bag = rosbag.Bag('test.bag', 'r')
		i=0
		
		
		twb_quat = [PoseStamped()]*2
		for (topic, msg, t) in bag.read_messages():

			Donnees.twb[i].pose.position.x = msg.pose.position.x
			Donnees.twb[i].pose.position.y = msg.pose.position.y
			Donnees.twb[i].pose.position.z = msg.pose.position.z

			twb_quat[i].pose.orientation.x = msg.pose.orientation.x
			twb_quat[i].pose.orientation.y = msg.pose.orientation.y
			twb_quat[i].pose.orientation.z = msg.pose.orientation.z
			twb_quat[i].pose.orientation.w = msg.pose.orientation.w
			i=i+1

			Donnees.Rwb[i] = tf.quaternion_matrix(twb_quat[i].pose.orientation.x, twb_quat[i].pose.orientation.y, twb_quat[i].pose.orientation.z, twb_quat[i].pose.orientation.w)
		 	print (Donnees.twb_quat)
		 	print (Donnees.Rwb)

 
		bag.close()
		rospy.spin()
#Ferme le bag

####################################
##############DEBUG#################
####################################

		#print Donnees.l
		#print ('\n')
		#print Donnees.x


####################################
###########PUBLIEUR ROS#############
####################################

	def talker(self, info):	
		hello_str = "%s" % info
		self.pub.publish(hello_str)


	def callback(self, msg): 
	    	self.talker(msg.data)
		rospy.loginfo(rospy.get_name())
	
if __name__ == "__main__":
    try:
        Capteur = main(Filename="test.bag")
    except rospy.ROSInterruptException:
        pass
