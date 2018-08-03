#!/usr/bin/python
# -*- coding: utf-8 -*-

import rosbag
import rospy
import numpy as np
import math as math	
import geometry_msgs
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32MultiArray
from rospy import init_node, is_shutdown
from parametres import Mesure
from tf import transformations as tf
from IPython import embed
import matplotlib.pyplot as plt

class main:

	def __init__(self):


		lidar_compt = raw_input("Entrez le numéro du lidar utilisé : \n")

#############################################################################################################################################
#TEST
		lidar_compt = 4
#############################################################################################################################################

		#Initiation du noeud , subscriber et publisher
	  	rospy.init_node('autocalib', anonymous=True)
		rospy.Subscriber('sub', String, self.callback)
		#self.pub = rospy.Publisher('chatter', String, queue_size=10)

		#Demande de chemins de bags
		(bag_mires, bag_mesures) = self.entree_chemins()
					
		#Lecture du mag incluant la détection des 2 mires et établissement des transformations entre les deux mires
		(translation_mires,rotation_mires) = self.lecture_des_normales(bag_mires)

		#Lecture de la longueur mesurée par rapport au monde
		tab_lidars = self.lecture_longueurs(bag_mesures)

		#Lecture de la position camera
		proxi = self.lecture_camera(bag_mesures)

		#Moyennage des valeurs de lidars autour des temps de position
		moyennage = self.moyennage_mesures_lidars(proxi, tab_lidars)		
		
		#moyennage, proxi = self.generation_mesures()
		#Calcul de la distance à t = 0 du capteur pour vérification de X
		#d0 = self.calcul_d0(translation_mires, proxi)

		bag_mesures.close()

		#Calcul du résultat x
		x = self.calcul_x(moyennage, lidar_compt, proxi, rotation_mires)

		print x




	####################################
	#############METHODES###############
	####################################

	
	def entree_chemins(self):
		chemin_deux_mires = "/home/denis/BAGS/Sixieme_serie_de_mesures/" + raw_input("Entrez le nom du bag comportant les deux mires au format \"NOM_DU_BAG.bag\" : \n")
		chemin_mesures = "/home/denis/BAGS/Sixieme_serie_de_mesures/" + raw_input("Entrez le nom du bag comportant les mesures lidar et camera au format \"NOM_DU_BAG.bag\" : \n")

#############################################################################################################################################
#TEST

		chemin_deux_mires = "/home/denis/BAGS/Premiere_serie_de_mesures/2mires.bag"
		chemin_mesures = "/home/denis/BAGS/Premiere_serie_de_mesures/mesure1.bag"
	#	chemin_deux_mires = "/home/denis/BAGS/Sixieme_serie_de_mesures/2mires_mire_simple.bag"
	#	chemin_mesures = "/home/denis/BAGS/Sixieme_serie_de_mesures/mesure_continue_mire_simple.bag"
#############################################################################################################################################

		#Bag de lecture de mesures
   		bag_mesures = rosbag.Bag(chemin_mesures, 'r')	
		#Lecture de la normale depuis le fichier 2mires.bag
   		bag_mires = rosbag.Bag(chemin_deux_mires, 'r')
		 
		return bag_mires, bag_mesures

	#Lecture du mag incluant la détection des 2 mires et établissement des transformations entre les deux mires
	def lecture_des_normales(self, _bag_mires):

		self.rotation_mires = []							# Rotation observée entre la mire horizontale et la verticale
		self.translation_mires = []							# Translation observée entre la mire horizontale et la verticale
		self.mires = []									# Transformation observée entre la mire horizontale et la verticale

		#Lecture de la normale depuis le fichier texte
		#		with open("/home/denis/catkin_ws/src/autocalibration/src/data.txt", "r") as f:
		#    			for line in f.readlines():
		#        			if 'Nw' in line:
		#					line= line.strip('\n')					
		#					y,z = line.split(':')
		#					z = z.split(',')
		#    			z1 = [float(i) for i in z]
		#			Nw_lect = np.array(z1)		


		#Mise en mémoire des détection des transformations cible caméra
		for (topic, msg, t) in _bag_mires.read_messages(topics = 'tag_detections'):

			self.mires.append([msg.detections[0].pose.pose.pose, msg.detections[1].pose.pose.pose])

		a=0
		#Transformation des données de rotation de quaternions en matrices et calcul des matrices de rotation sol-capteur et capteur-mur
		R_17_c = np.array(tf.quaternion_matrix([self.mires[a][0].orientation.x, self.mires[a][0].orientation.y, self.mires[a][0].orientation.z, self.mires[a][0].orientation.w]))
		R_c_21 = np.transpose(tf.quaternion_matrix([self.mires[a][1].orientation.x, self.mires[a][1].orientation.y, self.mires[a][1].orientation.z, self.mires[a][1].orientation.w]))


		#Réduction des matrices de rotation à 3 dimensions
		R_17_c = R_17_c[:3,:3]
		R_c_21 = R_c_21[:3,:3]

		#Calcul des translations sol-capteur et capteur-mur
		t_17_c = np.array([self.mires[0][0].position.x, self.mires[0][0].position.y, self.mires[0][0].position.z])
		t_21_c = np.array([self.mires[0][1].position.x, self.mires[0][1].position.y, self.mires[0][1].position.z])
		t_c_21 = - np.dot(R_c_21, t_21_c)

		#Calcul de rotation totale entre le sol et le mur
		self.rotation_mires = np.dot(R_c_21, R_17_c)
		
		#Addition des deux translations 
		self.translation_mires = np.add(t_17_c, -t_21_c)

		return self.translation_mires, self.rotation_mires



	#Calcul de la distance à t = 0 du capteur pour vérification de X
	def calcul_d0(self, _translation_mires, _proxi):


		self.R_21_c = np.array(tf.quaternion_matrix([_proxi[0][0].orientation.x, _proxi[0][0].orientation.y, _proxi[0][0].orientation.z, _proxi[0][0].orientation.w]))
		self.R_21_c = self.R_21_c[:3,:3]
		self.R_c_21 = np.transpose(self.R_21_c)
		
		distance_capteur_mur = _translation_mires[2] - np.dot(self.R_c_21, np.array([_proxi[0][0].position.x,_proxi[0][0].position.y, _proxi[0][0].position.z]))[2]


		#Calcul de la distance initiale du capteur au mur
		self.d0 = math.sqrt(np.square(t_c_17[0]) + np.square(t_c_17[1]) + np.square(t_c_17[2]))
		
		return d0






	#Lecture de la longueur mesurée par rapport au monde
	def lecture_longueurs(self, _bag):
		tab_lidars = []
		for (topic, msg, t) in _bag.read_messages(topics = 'mappys'):
			tab_lidars.append([msg.data, t.to_sec()])
		tab_lidars = np.array(tab_lidars)

		return tab_lidars

	#Lecture de la position camera
	def lecture_camera(self, _bag):
		proxi = []
		for (topic, msg, t) in _bag.read_messages(topics = 'tf'):
			proxi.append([msg.transforms[0].transform, msg.transforms[0].header.stamp.to_sec()])
		proxi = np.array(proxi)
		return proxi

	# Fonction de génération de paramètres entrés "en dur" pour test
	# Remplace les fonctions "lecture_camera", "lecture_longueurs" et "moyennage_mesures_lidars"
	def generation_mesures(self):

		proxi = []
		pose = []
		tab_lidars = []

		tab_lidars = [[[612],1532528096.470912000],[[909],1532528099.819173999],[[655],1532528107.665666999],[[984],1532528110.964180000],[[684],1532528117.11463000],[[649],1532528126.2574219999],[[353],1532528131.954766999]]

        	pose.append(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(-0.076414,-0.0138025,0.896131),geometry_msgs.msg.Quaternion(0.983468,-0.020823,-0.179758,-0.006613)))
	      	pose.append(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(-0.154615,0.268192,0.849482),geometry_msgs.msg.Quaternion(0.958493,-0.004453,-0.095945,0.268450)))
	   	pose.append(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0.126650,-0.1777319,0.885669),geometry_msgs.msg.Quaternion(0.949369,0.049062,-0.309852,-0.016838)))
	   	pose.append(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0.048553,-0.199161,0.846596),geometry_msgs.msg.Quaternion(0.883674,0.419044,0.200529,0.057537)))
	   	pose.append(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0.011198,-0.109262,1.123136),geometry_msgs.msg.Quaternion(0.975629,0.043485,-0.214231,-0.019044)))
	   	pose.append(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0.121662,-0.195784,0.828561),geometry_msgs.msg.Quaternion(0.989010,0.022398,0.144410,-0.022422)))
	   	pose.append(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(-0.131101,0.102438,0.686107),geometry_msgs.msg.Quaternion(0.987197,-0.022544,-0.130544,0.088841)))

		proxi = [[pose[0],1532528096.470912000],[pose[1],1532528099.819173999],[pose[2],1532528107.665666999],[pose[3],1532528110.964180000],[pose[4],1532528117.11463000],[pose[5],1532528126.2574219999],[pose[6],1532528131.954766999]]	
		
		self.moyennage = []
		for i in range (0,7):
			self.moyennage.append([])
			for j in range (0,7):
				if j == 4 :
					self.moyennage[i].append(tab_lidars[i][0][0])
				else : self.moyennage[i].append(0)

		return self.moyennage, proxi


	#Moyennage des valeurs de lidars autour des temps de position
	def moyennage_mesures_lidars(self, _proxi, _tab_lidars):
		self.Liste_moyennage = []							# Liste des valeurs à moyenner
												# 	Nombre de mesures lidars "synchrones"
												#		aux mesures camera 
												# 		  ............	
												#                |           |
												#                                  _    _
												# 	    _	 ____________   / | |    :  	 
												#  Nombre  :	 |_|_|_|_|_|_| /  | |    :  Nombre de
												#    de	   :	 |_|_|_|_|_|_|    | |    :   lidars
												#  mesures :	 |_|_|_|_|_|_| \  | |    :     6
												#  camera  :_	 |_|_|_|_|_|_|  \ |_|   _:  
												#    343                                        												#						

		self.moyennage = []								# Liste des valeurs moyennées
												# 		 Nombre de lidars	
												# 		  ............	
												#                |           |
												#                                 
												# 	    _	 ____________   	 
												#  Nombre  :	 |_|_|_|_|_|_| 
												#    de	   :	 |_|_|_|_|_|_|
												#  Mesures :	 |_|_|_|_|_|_|
												#  Camera  :_	 |_|_|_|_|_|_|
		

		for j in range(len(_proxi)):   							# Pour chaque mesure de camera... 
			self.Liste_moyennage.append([])		
			self.moyennage.append([])
			for i in range(len(_tab_lidars)):					# Pour chaque mesure de lidar...
#				print _tab_lidars[i][0]
				print np.shape(_proxi)
				if (abs(_tab_lidars[i][1] - _proxi[j][1]) < 0.025): #and _tab_lidars[i][0] != 0):		
#					embed()
												# Si le temps séparant les deux mesures est inferieur à 
												# 0.025s (demi période de la caméra) 
					self.Liste_moyennage[j].append(_tab_lidars[i][0])	# On liste toutes les mesures de lidar remplissant ce critère

#			if j == 3 or j== 5:  
#				self.Liste_moyennage[j] = []
#			if self.Liste_moyennage[j] == []:
#				np.delete(_proxi,j)

#			print self.Liste_moyennage
  			print np.shape(self.Liste_moyennage)
			for k in range(len(self.Liste_moyennage[j])):
				var = []	
#				embed()
				for l in range (len(self.Liste_moyennage[j])):
					var.append(self.Liste_moyennage[j][l][k])
				self.moyennage[j].append(np.mean(var))				# On fait leur moyenne de la liste ci-dessus sur les mesures 
												# de caméra
		self.moyennage = np.array(self.moyennage)			
		return self.moyennage	
	

	#Calcul du résultat x
	def calcul_x(self, _moyennage, _lidar_compt, _proxi, _rotation_mires):

		m = []										
		b = []
		x = [] 

		I = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]) 				# Matrice unité
		data = []
		for i in range(len(_moyennage)):

			l_lect = _moyennage[i][_lidar_compt]

			position_wb_lect = _proxi[i][0]

			#Remplissage structure de données
			Donnees = Mesure (position_wb_lect, l_lect, _rotation_mires)
			data += [Donnees]
			tbc = Donnees.tbc
			rbc = Donnees.Rbcrot

			twb = Donnees.twb
			rwb = Donnees.Rwbrot

			Nw = Donnees.Nw

			l = Donnees.l
			
			m.append(np.concatenate((np.matmul(np.transpose(Nw), rwb) * l/1000.0, np.matmul(np.transpose(Nw), (rwb-I)),[-1])))
			b.append(np.dot(Nw,twb))

		m = np.array(m)	
		b = np.array(b)	
		x = np.linalg.lstsq(m, -b)
		x1,x2,x3,x4 = np.linalg.lstsq(m, -b)

		err = np.dot(m,x1) - b
		
		#print err

		return x


	#Définition des communications ROS
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
