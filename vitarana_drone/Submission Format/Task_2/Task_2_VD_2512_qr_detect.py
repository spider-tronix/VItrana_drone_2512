#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from std_msgs.msg import Float32
import rospy

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode 
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		
		self.bridge = CvBridge()

		x_coords_pub = rospy.Publisher('/final_coordinates_x',Float32,queue_size=10)
		y_coords_pub = rospy.Publisher('/final_coordinates_y',Float32,queue_size=10)
		z_coords_pub = rospy.Publisher('/final_coordinates_z',Float32,queue_size=10)



	# Callback function of amera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			reader(self.img)

			
		except CvBridgeError as e:
			print(e)
			return

def reader(image):
	   
    detector = cv2.QRCodeDetector()
    data, bbox, straight_qrcode = detector.detectAndDecode(image)
    coords = data.split(',')
    print coords
    print len(coords)
    if(len(coords)>1):
	    x_coords_pub.publish(float(coords[0]))
	    y_coords_pub.publish(float(coords[1]))
	    z_coords_pub.publish(float(coords[2]))
    print coords[0]
    return coords

	

if __name__ == '__main__':
	image_proc_obj = image_proc()
	global x_coords_pub, y_coords_pub ,z_coords_pub
	x_coords_pub = rospy.Publisher('/final_coordinates_x',Float32,queue_size=10)
	y_coords_pub = rospy.Publisher('/final_coordinates_y',Float32,queue_size=10)
	z_coords_pub = rospy.Publisher('/final_coordinates_z',Float32,queue_size=10)
	rospy.spin()