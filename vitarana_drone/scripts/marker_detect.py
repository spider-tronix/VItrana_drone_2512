#!/usr/bin/env python


import  cv2 as cv
from  matplotlib import pyplot as plt
from std_msgs.msg import Float32
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys
from std_msgs.msg import Float64
from std_msgs.msg import Int8



# Node that detects the marker and publish the location of marker !!

class image_proc():

	
	def __init__(self):
		rospy.init_node('marker_test') #Initialise rosnode 
		

	# Callback function of camera topic


	def detect(self):
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
		self.z_m =0 
		self.build_num=0
		rospy.Subscriber('/z_m' , Float64 , self.z_callback)
		rospy.Subscriber('/build_num' , Int8 , self.build_num_callback)
		rospy.Subscriber('check_4',Int8 , self.check_callback)


		self.X_pub = rospy.Publisher('/x_marker', Float64 , queue_size=1)
		self.Y_pub = rospy.Publisher('/y_marker', Float64 , queue_size=1)


	def check_callback(self,msg):
		self.check = msg.data

	def z_callback(self,msg):
		self.z_m = msg.data

	def build_num_callback(self,msg):
		self.build_num = msg.data

	def image_callback(self, data):
		
		try:
			if(self.z_m!=0):
				
				self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
				self.image1 = cv.imread('/home/vivekubuntu/eyrc_task0/src/vitarana_drone/scripts/test_1.png')
				
				self.logo_cascade = cv.CascadeClassifier('/home/vivekubuntu/eyrc_task0/src/vitarana_drone/scripts/cascade.xml')
				self.gray = cv.cvtColor(self.img ,cv.COLOR_BGR2GRAY)


				self.logo = self.logo_cascade.detectMultiScale(self.gray, scaleFactor=1.05 , minNeighbors=2)

				for (x, y, w, h) in self.logo:

					cv.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
					print x+w/2 - 200
					print y+h/2 - 200
					self.focal_length = 238.350719
					# after the marker is detected this data is published so that controller node can generate the error
					self.X_pub.publish(x+w/2 - 200)
					self.Y_pub.publish(y+w/2 - 200)
				

			
			
		except CvBridgeError as e:
			print(e)
			return
 


	

if __name__ == '__main__':
	image_proc_obj = image_proc()
	r = rospy.Rate(100) 
	while not rospy.is_shutdown():      
		try:
		 image_proc_obj.detect()      
		 r.sleep()
		except rospy.exceptions.ROSTimeMovedBackwardsException: pass 
	
