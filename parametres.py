#!/usr/bin/python
# -*- coding: utf-8 -*-


from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from rospy import init_node, is_shutdown
from std_msgs.msg import String, Float32MultiArray
from rospy.numpy_msg import numpy_msg
from tf import transformations as tf
import numpy as np
import math as math	
import rospy


#  Schéma demonstratif des différentes transformations.
#
# Les rotations sont indiquées à l'extrémités de leurs axes.
#
#
#
#       ^
#     y |
#	|
#	|	    Rkhi  _ 	  / Rtheta
#	|		   -_	 /
#	|		     -_ / (c[i])	
#	|		       0
#	|		    Rphi	
#	|
#	|		      |
#	|
#	|		      		
#	|		    tbc 		
#	|	   Rgama      	
#	|	       \     	  _ Rbeta
#	|		\   |   _-
#	|		 \    _-   (b[i])
#	|		  \ _-
#	|		   0	
#	|		Ralpha
#	|	       -
#	|	     -
#	|	   -
#	|	twb
#	|     -
#	|  -
#     z 0_______________________________________________>
#		(w)		 			x

##################################################
#DEFINITION ET INITIATION DES MATRICES ET VECTEURS
##################################################

class Definitions :

	def __init__(self):

		#  Direction de la mesure
		self.e3	= Pose().position
		#  Vecteur normal au plan
		self.Nw = Pose().position
		# Vecteur de translation entre le monde et la base du drone
	
		self.twb = [Pose().orientation] * 7
		# Vecteur de rotation entre le monde et la base du drone en quaternion		
		self.RwbQuat = [Pose().orientation] * 7
		# Vecteur de rotation entre le monde et la base du drone en degrés	
		self.RwbDeg = np.array([] * 7)
		
		for i in range(1,7):
			np.append(self.RwbDeg, tf.quaternion_matrix([self.RwbQuat[i].x, self.RwbQuat[i].y, self.RwbQuat[i].z, self.RwbQuat[i].w]))

			
		#  Translation du centre du drone vers le capteur
		self.tbc = Pose().position
		#  Rotation du centre du drone vers le capteur en quaternion	
		self.RbcQuat = Pose().orientation
		#  Rotation du centre du drone vers le capteur en degrés
		RbcDeg = tf.quaternion_matrix([self.RbcQuat.x, self.RbcQuat.y, self.RbcQuat.z, self.RbcQuat.w])

		#  Distance reelle du capteur à l'origine
		self.dco0 = 5
		#  Matrice unité
		self.I = np.array([np.empty(3, float)]*3) 	
		#  Distance mesurée du capteur au plan
		self.l = np.empty(7, float)
		#  Matrice de vecteurs de situation concaténés
		self.x = np.empty(7, float)
		#  Vecteur du second membre de l'équation (position du drone) 
		self.b = np.empty(7, float)
		# Matrice de mesure
		self.m = np.array([np.empty(7, float)] * 7)

   	@property

	def setRwb (self, _Rwb):
		self.RwbQuat = tf.quaternion_matrix(_Rwb)

	def setRwb (self, _Rwb):
		self.RwbQuat = tf.quaternion_matrix(_Rbc)

	def settwb (self, _twb):
		self.twb = _twb

	def settbc (self, _tbc):
		self.tbc = _tbc

	def settbc (self, _Nw):
		self.Nw = _Nw

