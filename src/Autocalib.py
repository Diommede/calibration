#!/usr/bin/python
# -*- coding: utf-8 -*-

import rosbag
import rospy
import numpy as np
import math as math	
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32MultiArray
from rospy import init_node, is_shutdown
from generateur_de_mesure import generateur
from parametres import Definitions



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

		for i in range(0,8):
			Donnees.m[i] = np.concatenate((np.matmul(np.transpose(Donnees.Nw),Donnees.Rwb[i])*Donnees.l[i],np.matmul(Donnees.Nw,(Donnees.Rwb[i]-Donnees.I)),[-1]))
			Donnees.b[i] = np.dot(Donnees.Nw,Donnees.twb[i])
	
		Donnees.x = np.linalg.lstsq(Donnees.m, -Donnees.b)

######################################
################ROS###################
######################################

#Initiation du noeud , subscriber et publisher
	  	rospy.init_node('autocalib', anonymous=True)
		rospy.Subscriber('sub', String, self.callback)
		self.pub = rospy.Publisher('chatter', String, queue_size=10)

#Crée un bag en mode écriture
   		bag = rosbag.Bag('test.bag', 'w')


#Crée une série de 7 structures de mesures
		#self.message = [1.0]*7
		#self.message[0] = np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
		#for i in range(1,7):
		#	self.message.append(1)
		#	self.message[i] = np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])


#		self.topics = bag.get_type_and_topic_info()[1].keys()
#		print self.topics

#Ecriture du bag pour test

		mot =  Float32MultiArray()
		mot2 = Float32MultiArray()
		mot.data = [4.2, 4.1]
		mot2.data = [4.3, 3.1]
		bag.write('/coucou', mot2)
		bag.write('/coucou', mot)

		# calib
		#param normal du plan
                bag.write('/dist', mot2)
		bag.write('/pose', mot)
		-> fichier de config avec les param R_bc t_bc

                # normal
                lire le fichier + topic '/dist'

		#bag.close()
   		#bag = rosbag.Bag('test.bag', 'r')
		for (topic, msg, t) in bag.read_messages():
			print (topic, msg, t)
 
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
