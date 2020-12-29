#! /usr/bin/env python
# This code is a combination form turtlebot3_pointop_key and turtlebot3_obstacle. 
# We used both codes in additional to the action server that we created to recieve the mission.

import rospy
import actionlib
import autonomous_robots.msg
from time import sleep
from math import radians, copysign, sqrt, pow, pi, atan2, isnan
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from tf.transformations import euler_from_quaternion
import numpy as np
from angles import normalize_angle

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
robotname= "robot3" #Change this value 
framePre=robotname+"_tf/"
nameSpace=robotname+"/"
odom_frame = framePre + 'odom'
base_frame = framePre + 'base_footprint'
obsticledis = 0.3

def get_scan():
    scan = rospy.wait_for_message(nameSpace+'scan', LaserScan)
    scan_filter = []
    samples = len(scan.ranges)  # The number of samples is defined in 
                                # turtlebot3_<model>.gazebo.xacro file,
                                # the default is 360.
    samples_view = 1            # 1 <= samples_view <= samples
    
    if samples_view > samples:
        samples_view = samples
   
    myRange=15
    if samples_view == 1:
        scan_filter.extend(scan.ranges[:myRange])  
        scan_filter.extend(scan.ranges[-myRange:])  
        

    else:
        left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
        right_lidar_samples_ranges = samples_view//2
        
        left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
        right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
        scan_filter.extend(left_lidar_samples + right_lidar_samples)
    for i in range(samples_view):
        if scan_filter[i] == float('Inf'):
            scan_filter[i] = 3.5
        elif isnan(scan_filter[i]):
            scan_filter[i] = 0
    
    return scan_filter

def get_odom(tf_listener):
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)

    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception 222")
        return

    return (Point(*trans), rotation[2])

def rotate(goal_z, tf_listener, move_cmd, cmd_vel, position):
    goal_z = normalize_angle(np.deg2rad(goal_z))
    (position, rotation) = get_odom(tf_listener)
    r = rospy.Rate(10)
    while abs(rotation - goal_z) > 0.05:
            (position, rotation) = get_odom(tf_listener)
            print("rotation ", rotation)
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            cmd_vel.publish(move_cmd)
            r.sleep()
    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())

def goToGoal(goal_x, goal_y, tf_listener, move_cmd, cmd_vel, position):
    (position, rotation) = get_odom(tf_listener)
    last_rotation = 0
    linear_speed = 1
    angular_speed = 1
    r = rospy.Rate(10)
    goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
    distance = goal_distance

    while distance > 0.05:
        (position, rotation) = get_odom(tf_listener)
        x_start = position.x
        y_start = position.y
        path_angle = atan2(goal_y - y_start, goal_x- x_start)

        if path_angle < -pi/4 or path_angle > pi/4:
            if goal_y < 0 and y_start < goal_y:
                path_angle = -2*pi + path_angle
            elif goal_y >= 0 and y_start > goal_y:
                path_angle = 2*pi + path_angle
        if last_rotation > pi-0.1 and rotation <= 0:
            rotation = 2*pi + rotation
        elif last_rotation < -pi+0.1 and rotation > 0:
            rotation = -2*pi + rotation
        move_cmd.angular.z = angular_speed * path_angle-rotation

        distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
        move_cmd.linear.x = min(linear_speed * distance, 0.1)

        if move_cmd.angular.z > 0:
            move_cmd.angular.z = min(move_cmd.angular.z, 1.3)
        else:
            move_cmd.angular.z = max(move_cmd.angular.z, -1.3)

        last_rotation = rotation
        
        #Salam addd this
        lidar_distances = get_scan()
        front_distances =min(lidar_distances)
        print("####The distance is: ",front_distances)

        if (front_distances < obsticledis):
            print("Hitting Wall !!!!!!!!!!!!!!!")
            rospy.loginfo("Stop and Change Dircetion")
            cmd_vel.publish(Twist()) 
            (position, rotation) = get_odom(tf_listener)

            path_angle = atan2(goal_y - position.y, goal_x- position.x)
            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and position.y < goal_y:
                  path_angle = -2*pi + path_angle
                elif goal_y >= 0 and position.y > goal_y:
                    path_angle = 2*pi + path_angle
            print("the path_angle is ", path_angle)
            rotate(np.rad2deg(rotation)-90, tf_listener, move_cmd, cmd_vel, position)
            #exit()
            move_cmd.linear.x = 0.3
            move_cmd.angular.z = 0.05
            cmd_vel.publish(move_cmd)
            r = rospy.Rate(5)
            r.sleep() 
        
        else:    
            cmd_vel.publish(move_cmd) #Here
            r.sleep() 

    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())

def execute( goal):
# Do lots of awesome groundbreaking robot stuff here
    (goal_x, goal_y, goal_z) = goal.x,goal.y,0
    print("Iam moving ", goal.x,goal.y)
    goToGoal(goal_x, goal_y, tf_listener, move_cmd, cmd_vel, position)
    rotate(goal_z, tf_listener, move_cmd, cmd_vel, position)

    #sleep(10)
    print("Done")
    return 1
    #server.set_succeeded()
    


if __name__ == '__main__':
  rospy.init_node('Go_to_Goal_Server_'+robotname)
 
  cmd_vel = rospy.Publisher(nameSpace+'cmd_vel', Twist, queue_size=5)
  position = Point()
  move_cmd = Twist()
  r = rospy.Rate(10)
  tf_listener = tf.TransformListener()
  try:
    tf_listener.waitForTransform(odom_frame, base_frame, rospy.Time(), rospy.Duration(1.0))
  except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
    rospy.signal_shutdown("tf Exception 000")
  
  
  server = actionlib.SimpleActionServer(nameSpace+'goToGoal', autonomous_robots.msg.goToGoalAction, execute, False)
  server.start()
  rospy.spin()
