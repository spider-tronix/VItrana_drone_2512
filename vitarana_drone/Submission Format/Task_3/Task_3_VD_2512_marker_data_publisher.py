#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from vitarana_drone.msg import *


# this is publisher node which is publishing the required marker data 
# Builing number and height is subscribed fron controller node to publish the required values
class data_publisher():

	def __init__(self):
		rospy.init_node('marker_data_publisher') 

		self.z_m =0
		self.build_num =0
		self.X_marker =0 
		self.Y_marker =0
		self.check = 0
		self.error_x=0
		self.error_y=0

	def publish(self):

		
		rospy.Subscriber('/z_m' , Float64 , self.z_callback)
		rospy.Subscriber('/build_num' , Int8 , self.build_num_callback)
		rospy.Subscriber('/x_marker',Float64 , self.x_marker_callback)
		rospy.Subscriber('/y_marker',Float64 , self.y_marker_callback)
		rospy.Subscriber('check_4',Int8 , self.check_callback)

		rospy.Subscriber('/err_x' , Float64 , self.error_x_callback)
		rospy.Subscriber('/err_y' , Float64 , self.error_y_callback)

		self.focal_length = 238.350719
		self.marker_pub = rospy.Publisher('edrone/marker_data' , MarkerData , queue_size=1)
		self.marker_data = MarkerData()
		self.marker_data.marker_id= self.build_num

		if(self.check==1):
			# calculating using formulam
			self.marker_data.err_x_m = self.z_m*self.X_marker/self.focal_length
			self.marker_data.err_y_m = -self.z_m*self.Y_marker/self.focal_length
		else:
			# if drone is not detecting marker then publish Nan Values 
			self.marker_data.err_x_m = float("NaN")
			self.marker_data.err_y_m = float("NaN")

		self.marker_pub.publish(self.marker_data)
	
	def check_callback(self,msg):
		self.check = msg.data

	def z_callback(self,msg):
			self.z_m = msg.data

	def build_num_callback(self,msg):
			self.build_num = msg.data

	def x_marker_callback(self,msg):
	        self.X_marker = msg.data

	def y_marker_callback(self,msg):
	        self.Y_marker = msg.data

	def error_x_callback(self,msg):
		self.error_x = msg.data

	def error_y_callback(self,msg):
		self.error_y = msg.data


if __name__ == '__main__':

	marker_data_obj = data_publisher()
	
	r = rospy.Rate(12) 
	while not rospy.is_shutdown():      
		try:
		 marker_data_obj.publish()      
		 r.sleep()
		except rospy.exceptions.ROSTimeMovedBackwardsException: pass 