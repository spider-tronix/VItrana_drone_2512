#!/usr/bin/env python

import csv
import math
#import rospy


delivery = [[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0]]  #------------------------------------------list to store x,y,altitude,distance from start and name of deliveries
Return_init = [[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],
               [0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]  #-------------------list to store x,y,altitude and name of returns in given order
Return_final = [[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]  #------------------list to store x,y,altiutde and name of arranged ordered returns
trip_dist = [0,0,0,0,0,0,0,0,0]  #-------------------------------------------sum of diatnaces of each pair of return and delivery to start point 

temp = [0,0,0,0,0,0,0,0,0]  #------------------------------------------------variable to store temporary values while sorting

i=0   #----------------------------------------------------------------------incrementing variable for reading delivery details from manifest.csv
j=0   #----------------------------------------------------------------------incrementing variable for reading return details from manifest.csv

#rospy.init_node('scheduler_algo')

#--------functions to convert latitude and longitude to x and y respectively---------#

def lat_to_x(input_latitude):
    return 110692.0702932625 * (float(input_latitude) - 19)

def long_to_y(input_longitude):
    return -105292.0089353767 * (float(input_longitude) - 72)
#------------------------------------------------------------------------------------#


#--------functions to convert x and y to latitude and longitude respectively--------#

def x_to_lat(input_x):
    return (input_x/110692.0702932625)+19

def y_to_long(input_y):
    return (input_y/(-105292.0089353767))+72
#------------------------------------------------------------------------------------#


#-------function to convert string of location in .csv file to list-----------------#

def str2lst(string): 
    li = list(string.split(";")) 
    return li
#------------------------------------------------------------------------------------#


# -------------------STORE DELIVERY AND RETURN DETAILS FROM MANIFEST.CSV IN RESPECTIVE VARIABLES-------------------------------------------------------#
with open("/home/surya/Downloads/manifest.csv",'r') as file:
            csv_f = csv.reader(file)
            for row in csv_f:
                if (row[0]=="DELIVERY"):
                    delivery[0][i] = lat_to_x(str2lst(row[2])[0])             #---split the string and store the latitude as x
                    delivery[1][i] = long_to_y(str2lst(row[2])[1])            #---split the string and store the longitude as y
                    delivery[2][i] = str2lst(row[2])[2]                       #---split the string and store the altitude
                    delivery[4][i] = row[1]
                    delivery[3][i] = math.sqrt(((delivery[0][i]+19.5)*(delivery[0][i]+19.5))+((delivery[1][i]+16.5)*(delivery[1][i]+16.5)))  #---diatance of each delivery from centre of mat
                    i+=1
                if (row[0]=="RETURN "):
                    Return_init[0][j] = lat_to_x(str2lst(row[1])[0])          #---split the string and store the latitude as x
                    Return_init[1][j] = long_to_y(str2lst(row[1])[1])         #---split the string and store the longitude as y
                    Return_init[2][j] = str2lst(row[1])[2]                    #---split the string and store the altitude
                    Return_init[3][j] = row[2]
                    j+=1


# ----------------arranging deliveries in ascending order of their distances from start point-----------------------------------------------------------------#
for i in range(0, 9):    
    for j in range(i+1, 9):    
        if(delivery[3][i] > delivery[3][j]): 
            temp[0] = delivery[3][i]
            temp[1] = delivery[0][i]
            temp[2] = delivery[1][i]
            temp[3] = delivery[2][i]
            temp[4] = delivery[4][i]
            delivery[3][i] = delivery[3][j]
            delivery[0][i] = delivery[0][j]
            delivery[1][i] = delivery[1][j]
            delivery[2][i] = delivery[2][j]
            delivery[4][i] = delivery[4][j]    
            delivery[3][j] = temp[0]
            delivery[0][j] = temp[1]
            delivery[1][j] = temp[2]
            delivery[2][j] = temp[3]
            delivery[4][j] = temp[4]


# ---------------------pair up delivery and return with the criteria of return nearer to delivery --------------------------------------------------#
for i in range(0,9):
    dist = 0
    dist_prev = 200
    for j in range(0, 9):
        dist = math.sqrt(((delivery[0][i]-Return_init[0][j])*(delivery[0][i]-Return_init[0][j])) 
                           + ((delivery[1][i]-Return_init[1][j])*(delivery[1][i]-Return_init[1][j])))  # ---distance of each return from particular delivery
        assigned = 0
        for w in range(i):
            if(Return_final[0][w]==Return_init[0][j]): # --------------------check if the return is already paired with previous delivery
                assigned = 1
        if ((dist<=dist_prev)and(assigned==0)):       #-----------------------if respecttive shortest return isnt paired yet, pair it to the particular dilivery
            Return_final[0][i] = Return_init[0][j]
            Return_final[1][i] = Return_init[1][j]
            Return_final[2][i] = Return_init[2][j]
            Return_final[3][i] = Return_init[3][j]
            dist_prev = dist


# -----------------------calculate the sum of distances of paired delivery and reutns from start point -----------------------------------------------# 
for i in range(0,9):
    trip_dist[i] = math.sqrt(((delivery[0][i]+19.5)*(delivery[0][i]+19.5)) + ((delivery[1][i]+16.5)*(delivery[1][i]+16.5)))
    trip_dist[i] = trip_dist[i] + math.sqrt(((Return_final[0][i]+5.5)*(Return_final[0][i]+5.5)) + ((Return_final[1][i]+16.5)*(Return_final[1][i]+16.5)))


#--------------------arrange the paired deliveries and returns in the order of their total trip distance (longest to shortest) ----------------------------#
for i in range(0, 9):    
    for j in range(i+1, 9):    
        if(trip_dist[i] < trip_dist[j]): 
            temp[0] = trip_dist[i]
            temp[1] = delivery[0][i]
            temp[2] = delivery[1][i]
            temp[3] = delivery[2][i]
            temp[4] = delivery[4][i]
            temp[5] = Return_final[0][i]
            temp[6] = Return_final[1][i]
            temp[7] = Return_final[2][i]
            temp[8] = Return_final[3][i]
            trip_dist[i] = trip_dist[j]
            delivery[0][i] = delivery[0][j]
            delivery[1][i] = delivery[1][j]
            delivery[2][i] = delivery[2][j]
            delivery[4][i] = delivery[4][j]
            Return_final[0][i] = Return_final[0][j]
            Return_final[1][i] = Return_final[1][j]
            Return_final[2][i] = Return_final[2][j]
            Return_final[3][i] = Return_final[3][j]   
            trip_dist[j] = temp[0]
            delivery[0][j] = temp[1]
            delivery[1][j] = temp[2]
            delivery[2][j] = temp[3]
            delivery[4][j] = temp[4]
            Return_final[0][j] = temp[5]
            Return_final[1][j] = temp[6]
            Return_final[2][j] = temp[7]
            Return_final[3][j] = temp[8]


# ----------------convert x and y to latitude and longitude respectively to append in .csv file --------------------------------------------------#
for i in range(0,9):
    delivery[0][i] = x_to_lat(delivery[0][i])
    delivery[1][i] = y_to_long(delivery[1][i])
    Return_final[0][i] = x_to_lat(Return_final[0][i])
    Return_final[1][i] = y_to_long(Return_final[1][i])


#------------------append paired deliveries and returns in order into .csv file ----------------------------------------------------------------# 
with open('/home/surya/Downloads/final.csv', 'a') as f_object: 
    writer_object = csv.writer(f_object) 
    for i in range(0,9):
        writer_object.writerow(['DELIVERY', delivery[4][i], str(delivery[0][i]) + ";" + str(delivery[1][i]) + ";" + str(delivery[2][i])])
        writer_object.writerow(['RETURN' + ' ', str(Return_final[0][i]) + ";" + str(Return_final[1][i]) + ";" + str(Return_final[2][i]),  Return_final[3][i]]) 
    f_object.close()