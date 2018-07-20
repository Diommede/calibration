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
		
		lidar_inter = 5
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

#Lecture de la longueur mesurée par rapport au monde
		tab_lidars = []
   		bag = rosbag.Bag('/home/denis/BAGS/mesure_continue.bag', 'r')
		for (topic, msg, t) in bag.read_messages(topics = 'mappys'):
			tab_lidars.append([msg.data, t.to_sec()])
		tab_lidars = np.array(tab_lidars)

#Lecture de la position camera
#tag detection periode = 50ms
		proxi = []
		for (topic, msg, t) in bag.read_messages(topics = 'tf'):
			proxi.append([msg.transforms[0].transform, msg.transforms[0].header.stamp.to_sec()])
		proxi = np.array(proxi)
#Moyennage des valeurs de lidars autour des temps de position
		Liste_moyennage = []								# Liste des valeurs à moyenner
												# 	Nombre de mesures lidars "synchrones"
												# 		  ............	
												#                |           |
												#                                  _    _
												# 	    _	 ____________   / | |    :  	 
												#  Nombre  :	 |_|_|_|_|_|_| /  | |    :  Nombre de
												#    de	   :	 |_|_|_|_|_|_|    | |    :   lidars
												#  mesures :	 |_|_|_|_|_|_| \  | |    :     6
												#  camera  :_	 |_|_|_|_|_|_|  \ |_|   _:  
												#    343                                        												#						

		moyennage = []									# Liste des valeurs moyennées
												# 		 Nombre de lidars	
												# 		  ............	
												#                |           |
												#                                 
												# 	    _	 ____________   	 
												#  Nombre  :	 |_|_|_|_|_|_| 
												#    de	   :	 |_|_|_|_|_|_|
												#  Mesures :	 |_|_|_|_|_|_|
												#  Camera  :_	 |_|_|_|_|_|_|
												
                                           
		for j in range(len(proxi)):   							# Pour chaque mesure de camera... 
			Liste_moyennage.append([])		
			moyennage.append([])										
			for i in range(len(tab_lidars)):					# Pour chaque mesure de lidar...
				if (abs(tab_lidars[i][1] - proxi[j][1]) < 0.025):		# Si le temps séparant les deux mesures est inferieur à 
												# 0.25s (demi période de la caméra) 
					Liste_moyennage[j].append(tab_lidars[i][0])		# On liste toutes les mesures de lidar remplissant ce critère

			for k in range(len(tab_lidars[1][0])):
				var = []
				for l in range (len(Liste_moyennage[j])):
					var.append(Liste_moyennage[j][l][k])			
				moyennage[j].append(np.mean(var))				# On fait leur moyenne de la liste ci-dessus sur les mesures 
												# de caméra
		moyennage = np.array(moyennage)											

		bag.close()

####################################
##############CALCULS###############
####################################	
#///////////////////////////////revoir l'initialisation de m
		m = []
		b = []
		x = [] 

		I = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]) 
 
		for i in range(len(moyennage)):
			l_lect = moyennage[i][lidar_inter]

			position_wb_lect = proxi[0][0]

			#Remplissage structure de données
			Donnees = Mesure (position_wb_lect, Nw_lect, l_lect)

			tbc = Donnees.tbc
			rbc = Donnees.Rbcrot

			twb = Donnees.twb
			rwb = Donnees.Rwbrot

			Nw = Donnees.Nw
			l = Donnees.l
		
			m.append(np.concatenate((np.matmul(np.transpose(Nw), rwb) * l, np.matmul(Nw, (rwb-I)),[-1])))
			b.append(np.dot(Nw,twb))

		m = np.array(m)	
		b = np.array(b)	

		x = np.linalg.lstsq(m, -b)

		print x

#		rospy.spin()
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
