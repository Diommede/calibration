#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import numpy as np
import math as math	
from tf import transformations as tf
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32MultiArray
from rospy import init_node, is_shutdown

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

	def __init__(self, Nbmesures, _dco0 = 0., _Rbc = np.array([0.,0.,0.]) , _tbc =  np.array([0.,0.,0.]) , _Nw = np.array([0.,0.,0.])), _Rwb =np.array([0.,0.,0.]), _twb = np.array([0.,0.,0.])):
		


#e3, Nw, twb, tbc, dco0, I, l, x, b, m,	 Rcococ, tcococ 

#  Direction de la mesure
 	        self.e3	= np.array([0.,0.,0.])							
#  Vecteur normal au plan
		self.Nw = _Nw				
# Vecteur de translation entre le monde et la base du drone
 		self.twb = _twb
# Vecteur de rotation entre le monde et la base du drone		
 		self.Rwb = _Rwb
#  Translation du centre du drone vers le capteur
		self.tbc = _tbc	
#  Rotation du centre du drone ve rs le capteur
		self.Rbc = _Rbc					
#  Distance reelle du capteur à l'origine
		self.dco0 = _dco0	                					
#  Matrice unité
	        self.I = np.array([[0,0,0],[0,0,0],[0,0,0]]) 
#  Distance mesurée du capteur au plan
		self.l = np.array([0.,0.,0.,0.,0.,0.,0.,0.])
#  Matrice de vecteurs de situation concaténés
		self.x = np.array([0.,0.,0.,0.,0.,0.,0.,0.])	
#  Vecteur du second membre de l'équation (position du drone) 
		self.b = np.array([0.,0.,0.,0.,0.,0.,0.,0.])
# Matrice de mesure
		self.m = [] 							
		#self.m[0] = np.array([0.,0.,0.,0.,0.,0.,0.,0.])
		for i in range(0,8):
			self.m.append(1)	
			self.m[i] = np.array([0.,0.,0.,0.,0.,0.,0.,0.])
# Matrice de rotation de co vers c
		self.Rcococ=[8]	
		for i in range(0,8):
			self.Rcococ[i] = np.matmul(np.transpose(self.Rbc), np.matmul(self.Rwb[i], self.Rbc))
# Matrice de translation de co vers c						
		self.tcococ=[8]						
		for i in range(0,8):
			self.tcococ[i] = np.matmul(np.transpose(self.Rbc), (self.twb[i]-self.tbc + np.matmul(self.Rwb[i],self.tbc))) 

   	@property

	def setRwb (self, _Rwb = PoseArray()):
		self.Rwb[i] = tf.quaternion_matrix([_Rwb.poses.orientation.x, _Rwb.poses.orientation.y, _Rwb.poses.orientation.z, _Rwb.poses.orientation.w]) 

#	def settwb (self, _twb = PoseArray()):

					
	 getValeursInit(self):
	       	print "Récupération des valeurs initiales"
       		print "\ntwb"
       		print self.twb
       		print "\nphi"
       		print self.phi
       		print "\ndco0"
       		print self.dco0
       		print "\ntbc"
       		print self.tbc
       		print "\nNw"
       		print self.Nw
       		print "\ntheta"
       		print self.theta
       		print "\ne3"
       		print self.e3
       		print "\nI"
      		print self.I

   	def getValeursResult(self):
       		print "Récupération des valeurs de résultat"
       		print "\ntcococ"
       		print self.tcococ
       		print "\nRcococ"
       		print self.Rcococ
       		print "\nRbc"
       		print self.Rbc
       		print "\nphir"
       		print self.phir
       		print "\nRwb"
       		print self.Rwb
       		print "\nthetar"
       		print self.thetar
       		print "\nm"
       		print self.m
       		print "\nb"
       		print self.b
       		print "\nl"
       		print self.l
	       	print "\n Valeur attendue de x"
       		print self.x

	
