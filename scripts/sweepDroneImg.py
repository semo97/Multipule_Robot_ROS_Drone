#!/usr/bin/env python
# This code is written based on turtlebot3_pointop_key.
# We add the image puplisher, altitude control and drone sweep

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from autonomous_robots.msg import DroneCoordinat   		#Customized msg made by us
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
from sensor_msgs.msg import Image 	# Image is the message type
from cv_bridge import CvBridge 	# Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from os.path import isfile, join
import os
import re
import random
fileLoccation="/home/semo/catkin_ws/src/autonomous_robots/scripts/frames/"
col_frames = os.listdir('/home/semo/catkin_ws/src/autonomous_robots/scripts/frames/')
  # sort file names
col_frames.sort(key=lambda f: int(re.sub('\D', '', f)))
#print(col_frames)
  # empty list to store the frames
col_images=[]
limit = 0
for i in col_frames:
  # read the frames
    img = cv2.imread(fileLoccation+i)
    col_images.append(img)
    limit += 1  # number of frames

def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

class GotoPoint():
    def __init__(self):
        rospy.init_node('Drone_Sweeping_Node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('drone/cmd_vel', Twist, queue_size=5)
        # self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'world'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_footprint")
            rospy.signal_shutdown("tf Exception")
    
        (altitude, gg) = self.get_altitude()
        print(altitude.z)
        while (altitude.z <2):
            move_cmd.linear.z = 0.5
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            (altitude, gg) = self.get_altitude()
            print(altitude.z)
        rospy.loginfo("Stopping the Drone...")
        self.cmd_vel.publish(Twist())


    def goToGoal(self,x,y):
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)

        (position, rotation) = self.get_odom()
        linear_speed = 1
        angular_speed = 1
        maxSpeed = 0.5
        #move_cmd.linear.z = 5
        
        ## Add for loop , after each step take snap shot and send corrdinate 
        goal_x, goal_y = x,y
        
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y


            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            xGoal=goal_x - x_start
            yGoal=goal_y - y_start

            
            move_cmd.linear.x = clamp(linear_speed * xGoal, -maxSpeed,maxSpeed)
            move_cmd.linear.y = clamp(linear_speed * yGoal, -maxSpeed,maxSpeed)
            
            self.cmd_vel.publish(move_cmd)
            r.sleep()


        rospy.loginfo("Stopping the Drone...")
        self.cmd_vel.publish(Twist())
        rospy.Rate(0.5).sleep()
        # print(range(len(col_images)))
        randomIng=random.randint(0,len(col_images)-1)
        print(randomIng)
        rospy.loginfo("pub iumage "+ str(randomIng)+" X"+str(x))
        imgPubl=br.cv2_to_imgmsg(col_images[randomIng])
        congestion_location = DroneCoordinat()
        congestion_location.data=imgPubl
        congestion_location.x=x
        congestion_location.y=y
        coorPub.publish(congestion_location)
        # imgPub.publish(br.cv2_to_imgmsg(col_images[randomIng]))

        # Send snapshot and cordinare


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def get_altitude(self):
        try:
            (trans1, _) = self.tf_listener.lookupTransform("base_footprint", "base_stabilized", rospy.Time(0))

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception ddddd")
            return

        return (Point(*trans1), 0)


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        #imgPub = rospy.Publisher('video_frames', Image, queue_size=10)
        coorPub = rospy.Publisher('drone/congestion_location', DroneCoordinat, queue_size=10)
        br = CvBridge()
        while not rospy.is_shutdown():
            print("msg")
            myclass= GotoPoint()

            MAX_X=5
            MAX_Y=5
            for j in range(MAX_X):
                for i in range(MAX_Y):
                    x=j
                    y=i
                    if(j%2):
                        y=MAX_Y-1-i
                    print(x,y)
                    myclass.goToGoal(x,y)

    except:
        rospy.loginfo("shutdown program.")

