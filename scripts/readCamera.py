#!/usr/bin/env python3

import rospy
from autonomous_robots.msg import DroneCoordinat
import cv2, cv_bridge
car_cascade = cv2.CascadeClassifier('/home/semo/catkin_ws/src/autonomous_robots/scripts/cars.xml') 

class DroneCamera:
        def __init__(self):
    

                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)
                self.image_sub = rospy.Subscriber('drone/congestion_location',
                DroneCoordinat, self.image_callback)

        def image_callback(self, msg):
                image = self.bridge.imgmsg_to_cv2(msg.data)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                cars = car_cascade.detectMultiScale(gray, 1.1, 1)
                for (x,y,w,h) in cars: 
                        cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)  
                cv2.imshow("window", image)
                cv2.waitKey(5)
rospy.init_node('drone_camera')
drone_camera = DroneCamera()
rospy.spin()
