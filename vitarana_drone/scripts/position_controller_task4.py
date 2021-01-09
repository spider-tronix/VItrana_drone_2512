#!/usr/bin/env python

# Import the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int8
import rospy
import time
import tf
import math
import numpy as np
import  cv2 as cv
from  matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control
        
        # initialize drone's latitude, longitude and altitude (start with origin values)
         # latitude, longitude and altitude values at origin
        self.current_position = [0,0,0]

        # initialize setpoint(destination) values of drone (latitude, longitude, altitude) as origin
         # latitude, longitude and altitude values at origin
        self.setpoint_position = [0,0,0]
        self.starting_position = [0,0,0]
         
        # variables those get updated to 1 when drones reaches desired position
           # checkpoint_1 updates when drone reaches almost 3m altitude
           # checkpoint_2 updates when dronee reaches almost 19.0000451704 latitude
        self.checkpoint_1 =0
        self.checkpoint_2 =0

        # Declaring command_value of message type edrone_cmd and initializing values
        self.command_value = edrone_cmd()
        self.command_value.rcRoll = 0.0
        self.command_value.rcPitch = 0.0
        self.command_value.rcYaw = 0.0
        self.command_value.rcThrottle = 0.0

         # ---------------------------adding other variables for pid here -------------------------------------------------

        self.error = [0,0,0]       # initialize current errors in position(gps values) w.r.t setpoints
        self.errorI = [0,0,0]      # initialize sum of errors (for integral) for latitude, longitude and altitude
        self.errorD = [0,0,0]      # initialize change in error (for derivative) for latitude, longitude and altitude
        self.prev_error = [0,0,0]  # initialize errors in position w.r.t setpoints in previous cycle of pid

         # initial setting of Kp, Kd and ki for [latitude, longitude, altitude]
        # after tuning and computing corresponding PID parameters
        self.Kp = [36,36,60]
        self.Ki = [0, 0, 0]
        self.Kd = [2300,2300,1300]

        self.obstacle_right = float('inf')
        self.obstacle_left = float('inf')
        self.obstacle_right = float('inf')
        self.obstacle_front = float('inf')
        
        # # This is the sample time in which you need to run pid
        self.sample_time = 0.060  # in seconds

        # publish /drone_command and /lat_error
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=16)
        self.lat_error_pub = rospy.Publisher('/lat_error',Float32,queue_size = 1)
        self.long_error_pub = rospy.Publisher('/long_error',Float32,queue_size = 1)
        self.alt_error_pub = rospy.Publisher('/alt_error',Float32,queue_size = 1)
        self.detect_pub = rospy.Publisher('/detect_now',Int8,queue_size=10)
        self.change_min = rospy.Publisher('/edrone/range_finder_top',LaserScan,queue_size=10)
        self.LaserScan_reading = LaserScan()

        self.deactivate_pub = rospy.Publisher('/deactivate_gripper', String , queue_size = 1)
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
        self.img = np.empty([]) # This will contain your image frame from camera
        self.bridge = CvBridge()


        self.following_wall =0

        self.moving_forward =0
        self.moving_left = 0  
        self.moving_right =0
        self.moving_back =0

        self.intact_to_wall = 0

        self.picked =0
        self.moving_vertically =0
        self.cruising =0
        self.count = 1
        self.check=0
        self.new_initial_setpoint = [0,0,0]
        self.multiplier = 10
        self.final_setpoint_position = [0,0,0]
        self.prev_lower_setpoint = 25
        self.way_count = 0
        self.reached_to_pick =0 
        self.setting_waypoints = 0
        self.going_back_to_warehouse =0
        self.going_to_building =0
        self.detected_in_right=0
        self.detected_in_left=0
        self.land_on_marker=0
        self.going_to_marker=0 
        self.marker_position = [0,0,0]
        self.redetecting_marker = 0
        self.dropping_box =0


        # Subscribe to /edrone/gps
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.range_finder_top_callback)
        rospy.Subscriber('/edrone/range_finder_bottom',LaserScan,self.range_finder_bottom_callback)
        rospy.Subscriber('/gripper_response',String , self.gripper_callback)
        rospy.Subscriber('/final_coordinates_x',Float32,self.x_callback)
        rospy.Subscriber('/final_coordinates_y',Float32,self.y_callback)
        rospy.Subscriber('/final_coordinates_z',Float32,self.z_callback)
        rospy.Subscriber('/x_marker',Float64 , self.x_marker_callback)
        rospy.Subscriber('/y_marker',Float64 , self.y_marker_callback)       


    #callbacks for getting location through QR code
    def x_callback(self,msg):
        self.final_setpoint_position[0]=self.lat_to_x(msg.data)

    def y_callback(self,msg):
        self.final_setpoint_position[1]=self.long_to_y(msg.data)
    
    def z_callback(self,msg):
        self.final_setpoint_position[2]=msg.data
    
    def gripper_callback(self,msg):
        if(msg.data=='true'):
            self.picked=1
        else:
            self.picked=0



    def range_finder_top_callback(self,msg):

        self.obstacle_front= msg.ranges[4]
        self.obstacle_right= msg.ranges[1]
        self.obstacle_back = msg.ranges[2]
        self.obstacle_left = msg.ranges[3]



    def range_finder_bottom_callback(self,msg):
        self.obstacle_bottom = msg.ranges[0]


    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)   
       # -----------------------------------------------call back function -------------------------------------------------------
    
    def x_marker_callback(self,msg):
        self.X_marker = msg.data

    def y_marker_callback(self,msg):
        self.Y_marker = msg.data


    def image_callback(self, data):
        
        try:              
                self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
        
        except CvBridgeError as e:
            print(e)
            return



    # gps call back function
    # this function gets executed each time when gps publishes /edrone/gps
    def gps_callback(self, msg):
        self.current_position[0] = self.lat_to_x(msg.latitude)
        self.current_position[1] = self.long_to_y(msg.longitude)
        self.current_position[2] = msg.altitude
               
       # --------------------------------------------------------------------------------------------------------------------------

    def go_to_building(self, building_pos):
         if(self.following_wall==0):
                if(self.setting_waypoints==0):
                    self.final_position = building_pos
                    self.a=self.current_position[0]
                    self.b=self.current_position[1]
                    self.start_point = [self.a,self.b,25]
                    self.setting_waypoints=1
                    self.way_count=0
                
                self.distance = (math.sqrt((self.final_position[0]-self.start_point[0])*(self.final_position[0]-self.start_point[0]) + (self.final_position[1]-self.start_point[1])*(self.final_position[1]-self.start_point[1])))
                self.no_of_waypoints = int(self.distance/10 + 2)
                self.lat_way_array = np.linspace(self.start_point[0],self.final_position[0],self.no_of_waypoints)
                self.long_way_array = np.linspace(self.start_point[1],self.final_position[1],self.no_of_waypoints)
                self.setpoint_position = [self.lat_way_array[self.way_count],self.long_way_array[self.way_count],25]
                if(abs(self.current_position[0]-self.lat_way_array[self.way_count])<2 and self.way_count<self.no_of_waypoints-1):
                    self.way_count = self.way_count  + 1
                    self.setpoint_position = [self.lat_way_array[self.way_count],self.long_way_array[self.way_count],25]

    def go_to_warehouse(self, warehouse_pos):
            if(self.following_wall==0):
                if(self.setting_waypoints==0):
                    self.final_position = self.warehouse_position
                    self.a=self.current_position[0]
                    self.b=self.current_position[1]
                    self.start_point = [self.a,self.b,25]
                    self.setting_waypoints=1
                    self.way_count=0
                
                self.distance = (math.sqrt((self.final_position[0]-self.start_point[0])*(self.final_position[0]-self.start_point[0]) + (self.final_position[1]-self.start_point[1])*(self.final_position[1]-self.start_point[1])))
                self.no_of_waypoints = int(self.distance/10 + 2)
                self.lat_way_array = np.linspace(self.start_point[0],self.final_position[0],self.no_of_waypoints)
                self.long_way_array = np.linspace(self.start_point[1],self.final_position[1],self.no_of_waypoints)
                self.setpoint_position = [self.lat_way_array[self.way_count],self.long_way_array[self.way_count],25]
                if(abs(self.current_position[0]-self.lat_way_array[self.way_count])<2 and self.way_count<self.no_of_waypoints-1):
                    self.way_count = self.way_count  + 1
                    self.setpoint_position = [self.lat_way_array[self.way_count],self.long_way_array[self.way_count],25]
    
    def landing_on_marker(self):
        self.focal_length = 238.350719
        self.setpoint_position[0] = 10*self.X_marker/self.focal_length + self.current_position[0]
        self.setpoint_position[1] = -(10*self.Y_marker/self.focal_length) + self.current_position[1]
        self.land_on_marker=0
        self.going_to_marker=1

    def detect(self):
        self.logo_cascade = cv.CascadeClassifier('/home/vivekubuntu/eyrc_task0/src/vitarana_drone/scripts/cascade.xml')
        self.gray = cv.cvtColor(self.img ,cv.COLOR_BGR2GRAY)


        self.logo = self.logo_cascade.detectMultiScale(self.gray, scaleFactor=1.05 , minNeighbors=2)

        for (x, y, w, h) in self.logo:

            cv.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            print x+w/2 - 200
            print y+h/2 - 200
            self.focal_length = 238.350719
            # after the marker is detected this data is published so that controller node can generate the error
            self.setpoint_position[0] = 10*(x+w/2-200)/self.focal_length + self.current_position[0]
            self.setpoint_position[1] = -(10*(y+h/2-200)/self.focal_length) + self.current_position[1]
            self.land_on_marker=0
            self.going_to_marker=1

    def obstacle_in_right(self):
         if(self.following_wall==1):
            if(self.obstacle_right<8 and self.obstacle_right>0.35):   
                self.moving_back = 1
                self.setpoint_position[1]= -1.5 + self.current_position[1]
                self.setpoint_position[0]= -4 + self.current_position[0] + self.obstacle_right

            if(self.obstacle_right == float('inf')):
                self.moving_back =0
                self.setpoint_position[0]= 1.5 + self.current_position[0]

            if(self.obstacle_front != float('inf') and self.obstacle_front>0.35):
                self.intact_to_wall = 1
                self.moving_right =1
                self.setpoint_position[1]= -4 + self.current_position[1] + self.obstacle_front

            if(self.obstacle_front == float('inf') and self.intact_to_wall==1 ):
                self.detected_in_right=0
                self.intact_to_wall = 0
                self.following_wall=0
                self.setting_waypoints=0

    def obstacle_in_left(self):
        if(self.following_wall==1):
                if(self.obstacle_left<8 and self.obstacle_left>0.3):   
                    self.moving_back = 1
                    self.setpoint_position[1]= -1.5 + self.current_position[1]
                    self.setpoint_position[0]= 6 + self.current_position[0] - self.obstacle_left

                if(self.obstacle_left == float('inf')):
                    self.moving_back =0
                    self.setpoint_position[0]= -1.5 + self.current_position[0]

                if(self.obstacle_front != float('inf') and self.obstacle_front>0.3):
                    self.intact_to_wall = 1
                    self.moving_left =1
                    self.setpoint_position[1]= -4 + self.current_position[1] + self.obstacle_front

               
                if(self.obstacle_front == float('inf') and self.intact_to_wall==1 ):
                    self.detected_in_left=0
                    self.intact_to_wall = 0
                    self.following_wall=0
                    self.setting_waypoints=0

                


    def pid(self):
       

        self.pick_position = [1.5,6,8.44]
        self.building_position = [51.8185,-10,16.66]
        self.warehouse_position  = [0,0,11]


        

        if(self.picked==0 and self.reached_to_pick==0):
            self.setpoint_position[0] = self.pick_position[0]
            self.setpoint_position[1] = self.pick_position[1]
            self.setpoint_position[2] = 11
            print "Going above the mat to pick box"

        
        if(abs(self.current_position[0]-self.setpoint_position[0])<0.1 and abs(self.current_position[1]-self.setpoint_position[1])<0.1 and self.going_to_marker==0 and self.picked==0):
            self.deactivate_pub.publish("False")
            self.setpoint_position = self.pick_position
            self.reached_to_pick =1
            self.Kp[2]=40
            print "Going Down on Mat to pick Box"

        
        if(self.picked==1 and self.following_wall==0 and self.going_to_marker==0 and self.land_on_marker==0):
            self.Kp[2] = 60
            self.setpoint_position = self.building_position
            self.multiplier = 10
            self.going_to_building =1
            print "setting_position is set to building position "


       
        if(self.going_to_building==1):
            self.go_to_building(self.building_position)
            print "go_to_building function is called"
            
        
        if(abs(self.building_position[0]-self.current_position[0])<0.2 and abs(self.building_position[1]-self.current_position[1])<0.2 and self.going_to_marker==0):
            self.setpoint_position = self.building_position
            self.setpoint_position[2] = self.building_position[2] + 10
            self.land_on_marker = 1
            self.going_to_building=0
            self.detect_pub.publish(1)
            print "Reached to building height is getting increased"
                

        
        if(self.land_on_marker==1 and abs(self.setpoint_position[2]-self.current_position[2])<0.05):
            self.detect_pub.publish(1)           
            self.detect()
            print "Reached 10 meter of height now detecting marker"


        
        if(self.going_to_marker==1 and abs(self.setpoint_position[0]-self.current_position[0])<0.1 and abs(self.setpoint_position[1]-self.current_position[1])<0.1 and self.redetecting_marker==0):
            self.detect_pub.publish(1)
            self.detect()
            self.redetecting_marker=1
            print "redecting marker"


        if(self.redetecting_marker==1 and abs(self.setpoint_position[0]-self.current_position[0])<0.05 and abs(self.setpoint_position[1]-self.current_position[1])<0.05 and self.dropping_box==0):
            self.setpoint_position[2] = self.building_position[2]
            self.dropping_box=1

        if(self.dropping_box==1 and abs(self.setpoint_position[2]-self.current_position[2])<0.5):
            self.deactivate_pub.publish("True")
            self.going_back_to_warehouse=1
            self.setting_waypoints=0


        
        if(self.going_back_to_warehouse==1):
            self.go_to_warehouse(self.warehouse_position)




        #obstacle avoidance algorithm
        if(self.obstacle_right < 8 and self.obstacle_right>0.35 and self.error[0]>0):
            self.following_wall=1
            self.detected_in_right=1

        if(self.detected_in_right==1):
            print "obstacle detected in right"
            self.obstacle_in_right()

        if(self.obstacle_left < 10 and self.obstacle_left>0.35 and self.error[0]<0):
            self.following_wall=1
            self.detected_in_left=1

        if(self.detected_in_left==1):
            print "obstacle detected in left"
            self.obstacle_in_left()

            
        print self.setpoint_position
        
        self.error[0] = (self.setpoint_position[0] - self.current_position[0])
        self.error[1] = (self.setpoint_position[1] - self.current_position[1])
        self.error[2] = (self.setpoint_position[2] - self.current_position[2])


        # Compute sum of errors (for integral) for latitide, longitude and altitude
        self.errorI[0] += self.error[0]
        self.errorI[1] += self.error[1]
        self.errorI[2] += self.error[2]

        
        
        # Compute change in errors (for derivative) for latitude, longitude and altitude
        self.errorD[0] = self.error[0] - self.prev_error[0]
        self.errorD[1] = self.error[1] - self.prev_error[1]
        self.errorD[2] = self.error[2] - self.prev_error[2]
        
        # Calculate the pid output required for latitude, longitude and altitude
        self.output_latitude = (self.error[0]*self.Kp[0])+(self.errorI[0]*self.Ki[0])+(self.errorD[0]*self.Kd[0])
        self.output_longitude = (self.error[1]*self.Kp[1])+(self.errorI[1]*self.Ki[1])+(self.errorD[1]*self.Kd[1])
        self.output_altitude = (self.error[2]*self.Kp[2])+(self.errorI[2]*self.Ki[2])+(self.errorD[2]*self.Kd[2])

        # Use this computed output value in the equations to compute the roll, pitch and throttle forr drone

        

        self.command_value.rcThrottle = self.output_altitude
        self.command_value.rcRoll = 1500 + self.output_latitude  
        self.command_value.rcPitch = 1500 - self.output_longitude
        
        #updating previous error of latitude, longitude and altitude
        self.prev_error[0]= self.error[0]
        self.prev_error[1]= self.error[1]
        self.prev_error[2]= self.error[2]
        
        # publishing the roll, pitch and throttle values ( which will be subscribed by attitude controller )
        self.cmd_pub.publish(self.command_value)             
        
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(16)  # specify rate in Hz based upon your desired PID sampling time
    while not rospy.is_shutdown():      # run pid() until rospy is shutdown
        try:
         
         e_drone.pid()                 # execute pid equation
         r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass    #return error is anything goees wwrong
