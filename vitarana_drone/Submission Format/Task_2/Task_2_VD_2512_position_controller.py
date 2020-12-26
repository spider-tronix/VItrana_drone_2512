#!/usr/bin/env python

# Import the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import String
import rospy
import time
import tf
import math

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
        self.Kp = [10,10,15.2]
        self.Ki = [0, 0, 0]
        self.Kd = [1800,1800, 1000]

        self.obstacle_right = float('inf')
        self.obstacle_left = float('inf')
        self.obstacle_right = float('inf')
        self.obstacle_front = float('inf')
        
        # # This is the sample time in which you need to run pid
        self.sample_time = 0.060  # in seconds

        # publish /drone_command and /lat_error
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=16)
        self.lat_error_pub = rospy.Publisher('/lat_error',Float32,queue_size = 1)


        self.following_wall =0
        self.moving_forward =0
        self.moving_left = 0  
        self.moving_right =0
        self.intact_to_wall = 0
        self.picked =0
        self.moving_vertically =0
        self.cruising =0
        self.count = 1
        self.check=0
        self.new_initial_setpoint = [0,0,0]
        self.multiplier = 6
        self.final_setpoint_position = [0,0,0]
        self.prev_lower_setpoint = 25


        # Subscribe to /edrone/gps
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.range_finder_top_callback)
        rospy.Subscriber('/edrone/range_finder_bottom',LaserScan,self.range_finder_bottom_callback)
        rospy.Subscriber('/gripper_response',String , self.gripper_callback)
        rospy.Subscriber('/final_coordinates_x',Float32,self.x_callback)
        rospy.Subscriber('/final_coordinates_y',Float32,self.y_callback)
        rospy.Subscriber('/final_coordinates_z',Float32,self.z_callback)
        #rospy.Subscriber('/pid_tuning_roll',PidTune , self.set_roll_pid)


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
        self.obstacle_front = msg.ranges[0]
        self.obstacle_right = msg.ranges[1]
        self.obstacle_back = msg.ranges[2]
        self.obstacle_left = msg.ranges[3]

    def range_finder_bottom_callback(self,msg):
        self.obstacle_bottom = msg.ranges[0]


    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)   
       # -----------------------------------------------call back function -------------------------------------------------------
    
    
    def set_roll_pid(self,roll):
        self.Kp[1] = roll.Kp*0.06
        self.Ki[1] = roll.Ki*0.00008
        self.Kd[1] = roll.Kd



    # gps call back function
    # this function gets executed each time when gps publishes /edrone/gps
    def gps_callback(self, msg):
        self.current_position[0] = self.lat_to_x(msg.latitude)
        self.current_position[1] = self.long_to_y(msg.longitude)
        self.current_position[2] = msg.altitude
               
       # --------------------------------------------------------------------------------------------------------------------------

    
    def pid(self):
        # writing PID algorithm for latitude, longitude and altitude
       
    ##### -------------------------------- logic for problem statement -----------------------------------------------------------#####
        
        #checkpoints are basically that checks whether it reaches to that setpoint(1 or 2) or not 
        
        '''
        # set setpoint of (altitude as 3m)
        if(self.setpoint_position[0]==0 and self.setpoint_position[1]==0 and self.setpoint_position[2]==0):
            rospy.sleep(1)
            self.setpoint_position[2] = 3.0
        
        
        # set setpoint of latitude to 19.0000451704       
        if(self.checkpoint_1==0 and self.current_position[2]>2.99):
            rospy.sleep(1)
            self.setpoint_position = [5,0,3]   
            self.checkpoint_1 =1

        
        # set setpoint of altitude to 0.31
             
        if(self.checkpoint_2==0 and self.current_position[0]>4.9999):
            rospy.sleep(1)
            self.setpoint_position = [5,0,0.31]
            self.checkpoint_2=1
        
        '''
    ##### -----------------------------------------------------------------------------------------------------------------------#####
        
         
        
        #algorithm for movement od Drone
        #variables are assigned as their name suggest their use


        if(self.following_wall == 0 and self.picked==0 and self.moving_vertically==0):
            #move towards Box if box is not picked up
            self.setpoint_position = [78,11,25]
            
        if(self.picked==1 and self.following_wall==0 and self.obstacle_left==float('inf') and self.check==0):
            self.a = self.current_position[0]
            self.b = self.current_position[1]
            self.multiplier = 10

            #after succesfully avoiding the obstacle seetup new initial position
            self.new_initial_setpoint[0] = self.a
            self.new_initial_setpoint[1] = self.b


            self.check=1 
            self.count=1         
        
        # Computing errors(for proportional) for latitide, longitude and altitude
        
        if(self.following_wall == 0 and self.picked==1):
            
            self.setpoint_position = [78,11,27]
            self.moving_vertically=0

        if(self.picked==1 and self.following_wall==0):
            

            #Dynamic Kp and Kd for Box picked up
            self.Kp[0] = 12
            self.Kp[1] = 12
            self.Kp[2] = 7
            self.Kd[2] = 500
            
            self.initial_setpoint_position = [78,11,25]
            
            print self.count
            self.prev_setpoint = [78,11,25]

            #if obstacle is avoided new initail setpoint for cruising
            if(self.check==1):
                self.prev_setpoint = self.new_initial_setpoint
                print "Hello"

            #Below two variables is basically cos(theta) and sin(theta) for the line in which they are moving
            self.latitude_single_meter = (self.final_setpoint_position[0]-self.prev_setpoint[0])/math.sqrt((self.final_setpoint_position[0]-self.prev_setpoint[0])*(self.final_setpoint_position[0]-self.prev_setpoint[0]) + (self.final_setpoint_position[1]-self.prev_setpoint[1])*(self.final_setpoint_position[1]-self.prev_setpoint[1]))
            self.longitude_single_meter = (self.final_setpoint_position[0]-self.prev_setpoint[1])/math.sqrt((self.final_setpoint_position[0]-self.prev_setpoint[0])*(self.final_setpoint_position[0]-self.prev_setpoint[0]) + (self.final_setpoint_position[1]-self.prev_setpoint[1])*(self.final_setpoint_position[1]-self.prev_setpoint[1]))
            
            #Below is algorithm for dynamically change the setpoint while crusing 
            self.setpoint_position = [self.prev_setpoint[0]+self.count*self.multiplier*self.latitude_single_meter,self.prev_setpoint[1]+self.count*self.multiplier*self.longitude_single_meter,25]
            self.next_setpoint = [self.setpoint_position[0]+self.multiplier*self.latitude_single_meter,self.setpoint_position[1]+self.multiplier*self.longitude_single_meter,25]
            if(self.current_position[0]-self.setpoint_position[0]<2):
                self.setpoint_position = self.next_setpoint
                self.count=self.count+1
            if(self.current_position[1]-self.final_setpoint_position[1]<2):
                self.setpoint_position[1]=self.final_setpoint_position[1]
            if(self.current_position[0]-self.final_setpoint_position[0]<5):
                self.setpoint_position[0]=self.final_setpoint_position[0]

        #If very close to goal then start moving downwards
        if(self.current_position[0]-self.final_setpoint_position[0]<0.2 and self.picked==1):
            self.Kp[0]=0
            self.Kp[1]=0
            self.Kp[2]=10

            self.setpoint_position[2] = self.final_setpoint_position[2]
                

                
                    
        

        #This code is to be executed when about to pick up the box        
        if(self.following_wall==0 and self.picked==0):
            
            if(self.current_position[0]<78.3 and self.current_position[0]>77.3 and self.current_position[1]<11.3 and self.current_position[1]>10.7):
                self.setpoint_position[2]=22.1599967919
                self.moving_vertically=1
        print self.obstacle_left
        
        
        # this code is basically obstacle avoider
        if(self.obstacle_left < 10):
            
            self.setpoint_position[1]= -1.5 + self.current_position[1]
            self.setpoint_position[0]= 8 + self.current_position[0] - self.obstacle_left
            self.following_wall==1

        if(self.obstacle_left==float('inf') and self.following_wall==1):
            
            self.following_wall=0

        

        # Laser was giving bit wrong values so I rely upon only one laser
        


            '''    
            if(self.obstacle_left== float('inf') and self.following_wall==1 and self.moving_forward==0):
                
                self.moving_left = 0
                self.moving_forward = 1
                self.setpoint_position[1] = self.current_position[1] - 2
                self.setpoint_position[0] = -2 

            if(self.moving_forward==1 and self.obstacle_front< 10 and self.obstacle_front > 0.5):
                self.intact_to_wall = 1
                



            if(self.obstacle_front == float('inf') and self.following_wall==1 and self.moving_forward==1 and self.moving_left==0 and self.intact_to_wall==1):
                
                self.moving_forward = 0
                self.moving_right = 1
                self.setpoint_position[0] = self.current_position[0] - 2
                self.setpoint_position[1] = -2 + self.current_position[1]       
            '''    

            

       



        print self.setpoint_position
        self.lat_error_pub.publish(self.error[0])

        
       
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
