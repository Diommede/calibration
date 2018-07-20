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
#//////////////////////////isinstance(objet, classe) pour détecter le type
	def __init__(self, _position_wb, _Nw, _l):


#POSE LUE DE BAG/FICHIER
		# Vecteur de rotation entre le monde et la base du drone en quaternion		
		self.position_wb = _position_wb

#NP_ARRAYS

		# Vecteur de rotation entre le monde et la base du drone en degrés	
		self.interm = np.array(tf.quaternion_matrix([self.position_wb.rotation.x, self.position_wb.rotation.y, self.position_wb.rotation.z, self.position_wb.rotation.w]))
		self.Rwbrot = np.transpose(np.array([[self.interm[0][0],self.interm[0][1],self.interm[0][2]],[self.interm[1][0],self.interm[1][1],self.interm[1][2]],[self.interm[2][0],self.interm[2][1],self.interm[2][2]]]))
		# Vecteur de translation entre le monde et la base du drone
		self.twb = -np.dot(self.Rwbrot, np.array([self.position_wb.translation.x, self.position_wb.translation.y, self.position_wb.translation.z]))
		#  Translation du centre du drone vers le capteur
		self.tbc = np.array([1.,0.,0.])
		#  Rotation du centre du drone vers le capteur en quaternion	
		self.Rbcrot = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]) 
		# Vecteur normal au plan
		self.Nw = _Nw
		# Distance mesurée du capteur au plan
		self.l = _l



