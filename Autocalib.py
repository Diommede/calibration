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
from parametres import Mesure
from tf import transformations as tf


####################################
############MAIN FUNCTION###########
####################################

class main:

	def __init__(self):

######################################
###############LECTURE################
######################################

#Initiation du noeud , subscriber et publisher
	  	rospy.init_node('autocalib', anonymous=True)
		rospy.Subscriber('sub', String, self.callback)
		self.pub = rospy.Publisher('chatter', String, queue_size=10)

#Lecture de la normale depuis le fichier texte
		with open("/home/denis/catkin_ws/src/autocalibration/src/data.txt", "r") as f:
    			for line in f.readlines():
        			if 'Nw' in line:
					line= line.strip('\n')					
					y,z = line.split(':')
					z = z.split(',')
    			z1 = [float(i) for i in z]
			Nw_lect = np.array(z1)		
					
#Lecture de la longueur mesurée et de la position par rapport au monde
		tab_lidars = []
   		bag = rosbag.Bag('/home/denis/BAGS/mesure2.bag', 'r')
		for (topic, msg, t) in bag.read_messages(topics = 'mappys'):
			tab_lidars.append([msg,t])
			
			print tab_lidars
		

		l_lect = msg.data[3]
		proxi = []
##tag detection periode = 50ms
#		for (topic, msg, t) in bag.read_messages(topics = 'tf'):
#			position_wb_lect = msg.transforms[0].transform

#	#Moyennage des valeurs de lidars autour des temps de position
#			while i < len(tab_lidars)
#				if  = tab_lidars[i][2]
#					proxi[i]
#			print e

		bag.close()
		rospy.spin()

####################################
##############CALCULS###############
####################################	
#///////////////////////////////revoir l'initialisation de m
		m = np.array([np.array([0.,0.,0.,0.,0.,0.,0.])*8])
		b = np.array([4.]*8)
		x = 0 

		I = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]) 

		#Remplissage structure de données
		print l_lect
		Donnees = Mesure (position_wb_lect, Nw_lect, l_lect)

		tbc = Donnees.tbc
		rbc = Donnees.Rbcrot
		
		twb = Donnees.twb
		rwb = Donnees.Rwbrot

		Nw = Donnees.Nw
		l = Donnees.l
				
		#m = np.concatenate((np.matmul(np.transpose(Nw), rwb) * l, np.matmul(Nw, (rwb-I)),[-1]))
		m = np.array([np.concatenate((np.matmul(np.transpose(Nw), rwb) * l, np.matmul(Nw, (rwb-I)),[-1]))]*8)
		np.append(b, np.dot(Nw,twb))
	
#		print m
#		print b


		x = np.linalg.lstsq(m, -b)
#		print x


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
        Capteur = main()
    except rospy.ROSInterruptException:
        pass
