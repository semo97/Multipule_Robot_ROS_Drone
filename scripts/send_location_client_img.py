#!/usr/bin/env python
# This code is written from scratch 

import sys
import rospy
import random as rn
from autonomous_robots.srv import *
import actionlib
#from autonomous_robots.msg import *
import autonomous_robots.msg
from math import radians, copysign, sqrt, pow, pi, atan2, isnan
import tf
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from time import sleep
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from autonomous_robots.msg import DroneCoordinat

# Trained XML classifiers describes some features of some object we want to detect 
car_cascade = cv2.CascadeClassifier('/home/semo/catkin_ws/src/autonomous_robots/scripts/cars.xml') 
frame_num =0
def imagecallback(mydata):
    br = CvBridge()
    rospy.loginfo("receiving frames")

    image = br.imgmsg_to_cv2(mydata.data)
    x_goal = mydata.x 
    y_goal = mydata.y
    # cv2.imshow("image",image)
    # cv2.waitKey(1)
    # convert to gray scale of each frames 
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    
    # Detects cars of different sizes in the input image 
    cars = car_cascade.detectMultiScale(gray, 1.1, 1) 
    
    # To draw a rectangle in each cars 
    number_of_centroid = 1
    for (x,y,w,h) in cars: 
        cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2) 
        number_of_centroid += 1

    # Display image in a window
    # Add rturning value
    if number_of_centroid > 5:

        print('sending goal: '+ str(x_goal)+ ","+str(y_goal)) 
        send_location_client(x_goal, y_goal)
        print("there is a Congestion, In: "+str(frame_num))
    else:
        print("No Congestion")

def send_location_client(x, y):
    print('Before Waiting')
    
    goal = autonomous_robots.msg.goToGoalGoal(x,y)
    n=[0,0,0]
    n[0]= actionlib.SimpleActionClient.get_state(client[0])
    n[1]= actionlib.SimpleActionClient.get_state(client[1])
    n[2]= actionlib.SimpleActionClient.get_state(client[2])
    
    if not n[0] == 1 : # and done task: also set state back to 0 after done
        # feedback
        client[0].send_goal(goal)
        print('sending goal to robot1')
    elif not n[1] == 1 :
        client[1].send_goal(goal)
        print('sending goal to robot2')
    elif not n[2] == 1 :
        client[2].send_goal(goal)
        print('sending goal to robot3')


    # Waits until the action server has started up and started
    # listening for goals.

    # Creates a goal to send to the action server.
    print('After Waiting')
    
    return 1


if __name__ == "__main__":
	# wait for cordinate from image pro server
    global client
    client=[0,0,0]
    client[0] = actionlib.SimpleActionClient('robot1/goToGoal', autonomous_robots.msg.goToGoalAction)
    client[1] = actionlib.SimpleActionClient('robot2/goToGoal', autonomous_robots.msg.goToGoalAction)
    client[2] = actionlib.SimpleActionClient('robot3/goToGoal', autonomous_robots.msg.goToGoalAction)

    # add spin
    try:
        # add if statment to selevt the robt 
        rospy.init_node('Go_to_Goal_client') 
        client[0].wait_for_server()
        client[1].wait_for_server()
        client[2].wait_for_server()
      
        #rospy.Subscriber('video_frames', Image, imagecallback)
        rospy.Subscriber('drone/congestion_location', DroneCoordinat, imagecallback)
        rospy.spin() 
    except rospy.ROSInterruptException:
        print("Action call failed")
