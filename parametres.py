#!/usr/bin/python
# -*- coding: utf-8 -*-


from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion, Point
from rospy import init_node, is_shutdown
from std_msgs.msg import String, Float32MultiArray
from rospy.numpy_msg import numpy_msg
from tf import transformations as tf
import numpy as np
import math as math	
import rospy


# Schéma demonstratif des différentes transformations.
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

class Mesure :
#//////////////////////////instance(objet, classe) pour détecter le type
	def __init__(self, _position_wb, _l, rot_matrix):

#POSE LUE DE BAG/FICHIER
		# Vecteur de rotation entre le monde et la base du drone en quaternion		
		self.position_wb = _position_wb
			
#NP_ARRAYS
		# Vecteur de rotation entre le monde et la base du drone en degrés	

		self.interm = np.array(tf.quaternion_matrix([self.position_wb.rotation.x, self.position_wb.rotation.y, self.position_wb.rotation.z, self.position_wb.rotation.w]))
		self.interm = self.interm[:3,:3]
		self.Rwbrot = np.transpose(self.interm)

		# Vecteur normal au plan du mur
		self.Nw = np.dot(rot_matrix,[0,0,1])


		# Vecteur de translation entre le monde et la base du drone
		self.twb = -np.dot(self.Rwbrot, np.array([self.position_wb.translation.x, self.position_wb.translation.y, self.position_wb.translation.z]))
		#  Translation du centre du drone vers le capteur
		self.tbc = np.array([1.,0.,0.])
		#  Rotation du centre du drone vers le capteur en quaternion	
		self.Rbcrot = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]) 
		# Distance mesurée du capteur au plan
		self.l = _l
