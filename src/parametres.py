#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import numpy as np
import math as math	

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

	def __init__(self, _e3 = np.array([0.,0.,0.]) , _twb = np.array([[np.random.uniform(0, 0),np.random.uniform(0., 0.),np.random.uniform(0., 0.)],[np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.)],[np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.)],[np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.)],[np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.)],[np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.)],[np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.)],[np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.)]]),  _alpha = np.array([np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.)]), _beta = np.array([np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),-np.random.uniform(0., 0.)]), _gama = np.array([np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.),np.random.uniform(0., 0.)]), _phi = 0., _theta = 0., _khi = -0., _dco0 = 0., _tbc = np.array([0.,0.,0.]), _Nw = np.array([0.,-0.,0.])):	
		
#alpha, alphar, beta, betar, gama, gamar: angles de rotation du monde vers le drone

#rotation du drone autour de z	
		self.alpha = _alpha						
		self.alphar = [8]
		self.Ralpha = [8]
		self.alphar[0] = math.pi/180*self.alpha[0]			#conversion en radians
		self.Ralpha[0] = np.array([[math.cos(self.alphar[0]),-math.sin(self.alphar[0]),0],[math.sin(self.alphar[0]),math.cos(self.alphar[0]),0],[0,0,1]]) 						
		for i in range(1,8):
			self.alphar.append(1)
			self.Ralpha.append(1)
			self.alphar[i] = math.pi/180*self.alpha[i]
			self.Ralpha[i] = np.array([[math.cos(self.alphar[i]),-math.sin(self.alphar[i]),0],[math.sin(self.alphar[i]),math.cos(self.alphar[i]),0],[0,0,1]]) 

#rotation du drone autour de y
		self.beta = _beta						
		self.betar = [8]		
		self.Rbeta = [8]
		self.betar[0] = math.pi/180*self.beta[0]			#conversion en radians
		self.Rbeta[0] = np.array([[math.cos(self.betar[0]), 0, math.sin(self.betar[0])], [0,1,0], [-math.sin(self.betar[0]), 0, math.cos(self.betar[0])]]) 						
		for i in range(1,8):
			self.betar.append(1)
			self.Rbeta.append(1)
			self.betar[i] = math.pi/180*self.beta[i]
			self.Rbeta[i] = np.array([[math.cos(self.betar[i]), 0, math.sin(self.betar[i])], [0,1,0], [-math.sin(self.betar[i]), 0, math.cos(self.betar[i])]]) 

#rotation du drone autour de x
		self.gama = _gama						
		self.gamar = [8]
		self.Rgama = [8]						
		self.gamar[0] = math.pi/180*self.gama[0] 			#conversion en radians
		self.Rgama[0] = np.array([[1,0,0],[0,math.cos(self.gamar[0]),-math.sin(self.gamar[0])],[0,math.sin(self.gamar[0]),math.cos(self.gamar[0])]]) 
		for i in range(1,8):
			self.gamar.append(1)
			self.Rgama.append(1)
			self.gamar[i] = math.pi/180*self.gama[i]
			self.Rgama[i] = np.array([[1,0,0],[0,math.cos(self.gamar[i]),-math.sin(self.gamar[i])],[0,math.sin(self.gamar[i]),math.cos(self.gamar[i])]]) 

#Rwb			
		self.Rwb = [8]							# Matrice de rotation entre le monde et la base du drone
		self.Rwb[0] = np.matmul(np.matmul(self.Ralpha[0], self.Rbeta[0]),self.Rgama[0])
		self.Rwb[0] = np.matmul(np.matmul(self.Ralpha[0], self.Rbeta[0]),self.Rgama[0])
		for i in range(1,8):
			self.Rwb.append(1)						
			self.Rwb[i] = np.matmul(np.matmul(self.Ralpha[i], self.Rbeta[i]),self.Rgama[i])

#phi, phir, theta, thetar, khi, khir: angles de rotation du drone vers le capteur

#rotation du capteur autour de z
		self.phi = _phi						
		self.phir = math.pi/180*self.phi	 			#conversion en radians
		self.Rphi = np.array([[math.cos(self.phir),-math.sin(self.phir),0],[math.sin(self.phir),math.cos(self.phir),0],[0,0,1]]) 

#rotation du capteur autour de y
		self.theta = _theta						
		self.thetar = math.pi/180*self.theta	 			#conversion en radians
		self.Rtheta = np.array([[math.cos(self.thetar), 0, math.sin(self.thetar)], [0,1,0], [-math.sin(self.thetar), 0, math.cos(self.thetar)]]) 
#rotation du capteur autour de x
		self.khi = _khi						
		self.khir = math.pi/180*self.khi				#conversion en radians
		self.Rkhi = np.array([[1,0,0],[0,math.cos(self.khir),-math.sin(self.khir)],[0,math.sin(self.khir),math.cos(self.khir)]]) 
	
		self.Rbc = np.matmul(np.matmul(self.Rkhi, self.Rtheta),self.Rphi)	#Rotation du faisceau du capteur par rapport au corps
	
#e3, Nw, twb, tbc, dco0, I, l, x, b, m,	 Rcococ, tcococ 

#  Direction de la mesure
 	        self.e3 = _e3								
#  Vecteur normal au plan
		self.Nw = _Nw							
# Vecteur de translation entre le monde et la base du drone
		self.twb = _twb 						
#  Translation du centre du drone vers le capteur
		self.tbc = _tbc						
#  Distance reelle du capteur à l'origine
		self.dco0 = _dco0	                					
#  Matrice unité
	        self.I = np.array([[1,0,0],[0,1,0],[0,0,1]]) 
#  Distance mesurée du capteur au plan
		self.l = np.array([0.,0.,0.,0.,0.,0.,0.,0.])
#  Matrice de vecteurs de situation concaténés
		self.x = np.array([0.,0.,0.,0.,0.,0.,0.,0.])	
#  Vecteur du second membre de l'équation (position du drone) 
		self.b = np.array([0.,0.,0.,0.,0.,0.,0.,0.])
# Matrice de mesure
		self.m = [8] 							
		self.m[0] = np.array([0.,0.,0.,0.,0.,0.,0.,0.])
		for i in range(1,8):
			self.m.append(1)	
			self.m[i] = np.array([0.,0.,0.,0.,0.,0.,0.,0.])
# Matrice de rotation de co vers c
		self.Rcococ=[8]	
		self.Rcococ[0] = np.matmul(np.transpose(self.Rbc), np.matmul(self.Rwb[0], self.Rbc))
		for i in range(1,8):
			self.Rcococ.append(1)
			self.Rcococ[i] = np.matmul(np.transpose(self.Rbc), np.matmul(self.Rwb[i], self.Rbc))
# Matrice de translation de co vers c						
		self.tcococ=[8]							
		self.tcococ[0] = np.matmul(np.transpose(self.Rbc), (self.twb[0]-self.tbc + np.matmul(self.Rwb[0],self.tbc) ) ) 	
		for i in range(1,8):
			self.tcococ.append(1)
			self.tcococ[i] = np.matmul(np.transpose(self.Rbc), (self.twb[i]-self.tbc + np.matmul(self.Rwb[i],self.tbc))) 

   	@property
#	def gettwb():
#		return self.twb
#	def getphi():
#		return self.phi
#	def getdco0():
#		return self.dco0
#	def gettbc():
#		return self.tbc
#	def getNw():
#		return self.Nw
#	def gettheta():
#		return self.theta
#	def gete3():
#		return self.e3
#	def getI():
#		return self.I
#	def getphi():
#		return self.phi
#	def gettcococ():
#		return self.tcococ
#	def getRcococ():
#		return self.Rcococ
#	def getRbc():
#		return self.Rbc
#	def getphir():
#		return self.phir
#	def getRwb():
#		return self.Rwb
#	def getthetar():
#		return self.thetar
#	def getm():
#		return self.m
#	def getb():
#		return self.b
#	def getx():
#		return self.x
#	def getl():
#		return self.l

#	def settwb(twb):
#		self.twb=twb
#	def setphi(phi):
#		self.phi=phi
#	def setdco0(dco0):
#		self.dco0=dco0
#	def settbc(tbc):
#		self.tbc=tbc
#	def setNw(Nw):
#		self.Nw=Nw
#	def settheta(theta):
#		self.theta=theta
#	def sete3(e3):
#		self.e3=e3
#	def setI(I):
#		self.I=I
#	def setphi(phi):
#		self.phi=phi
#	def settcococ(tcococ):
#		self.tcococ=tcococ
#	def setRcococ(Rcococ):
#		self.Rcococ=Rcococ
#	def setRbc(Rbc):
#		self.Rbc=Rbc
#	def setphir(phir):
#		self.phir=phir
#	def setRwb(Rwb):
#		self.Rwb=Rwb
#	def setthetar(thetar):
#		self.thetar=thetar
#	def setm(m):
#		self.m=m
#	def setb(b):
#		self.b=b
#	def setx(x):
#		self.x=x
#	def setl(l):
#		self.l =l



	def getValeursInit(self):
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

#	def settwb(e3):
#		self._e3 = e3
#	def settwb(twb):
#		self._twb = twb
#	def setphi(phi):
#		self._phi = phi
#	def settdco0(dco0):
#		self._dco0 = dco0
#	def settbc(tbc):
#		self._tbc = tbc
#	def setNw(Nw):
#		self._Nw = Nw
#	def settheta(theta):
#		self._theta = theta

	
