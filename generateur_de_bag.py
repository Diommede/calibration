#!/usr/bin/python
# -*- coding: utf-8 -*-


import rosbag
import rospy
import numpy as np
import math as math	
from std_msgs.msg import String, Float32MultiArray
from rospy import init_node, is_shutdown
#from generateur_de_mesure import generateur
#from parametres import Definitions



###################################
###########MAIN FUNCTION###########
###################################

class generateur_de_bag:
	def __init__(self):
		rospy.init_node('generateur', anonymous=True)
	  	bag = rosbag.Bag('test.bag', 'a')

		topics = bag.get_type_and_topic_info()[1].keys()
		print topics

	   	for topic, msg, t in bag.read_messages(topics =['/coucou']):
			print msg
		bag.close()
		rospy.spin()

	
if __name__ == "__main__":
    try:
        bagtowrite = generateur_de_bag()
    except rospy.ROSInterruptException:
        pass
