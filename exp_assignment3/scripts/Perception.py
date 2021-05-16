#!/usr/bin/env python

## @file robot_following.py
# @brief Perception algorithm which handles the ball tracking and the swing routine
#
# Details: This node implements the perception algorithm to track the ball from images received from the simulation environment
#

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import math

# Ros Messages
from sensor_msgs.msg import CompressedImage,JointState,LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

VERBOSE = False


## VARIABLES

## Initial angle definition
angle=0
## Threshold precision
threshold = 0.0001
## Dimension of the proximity radius
max_rad=125
## robot angle
rob_angle=0
## robot vel
rob_vel=0
## ROOMS PARAMETERS
names=["Entrance","Closet","Living_room","Kitchen","Bathroom","Bedroom"] ## rooms name
lower=[100,0,50,25,125,0] ## lower value in first position of hsv
upper=[130,5,70,35,150,5] ## upper value in first position of hsv
checked=[0]*len(names) ## boolean of whether the ball have already been seen
## Track activate variable
active=0
## Seen balls
seen=[0]*len(checked)
## Contours
cnts=[None]*len(seen)
## CALLBACKS AND FUNCTIONS

## callback function to update the robot position and orientation
def rob_callback(ros_data):
	global rob_angle,rob_vel
	temp=ros_data.pose.pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion ([temp.x, temp.y, temp.z, temp.w])
	rob_angle=yaw
	rob_vel=ros_data.twist.twist.linear.x

## find the enclosing circle of the ball in the image
def enclosing_circle(cnts):
    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    c = max(cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    return [x,y,radius,center]

def caption(image_np,x,y,radius,center,name):
    # draw the circle and centroid on the frame,
    # then update the list of tracked points
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
    cv2.circle(image_np, center, 5, (0, 0, 255), -1) 
    cv2.putText(image_np,name,(int(x),int(y)), font, 1,(255,255,255),2)   

## CLASS

## Image feature class to handle feature identification inside the image sent by the camera
class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/robot/joint1_position_controller/command",
                                         Float64, queue_size=1)
        # subscribed Topic
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",CompressedImage, self.callback,queue_size=1)
        rospy.Subscriber("/robot/odom",Odometry,rob_callback,queue_size=1)
		

    def callback(self, ros_data):
        global threshold, seen, checked, active
        ##Callback function of subscribed topic. 
        ##Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)


        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        ## reset which balls the robot sees
        seen=[0]*len(seen)
        center = None
        cnts=[None]*len(seen)

        ## Check all colors to see if a ball is in the image
        for i in range(len(names)):
            Lower = (lower[i], 50, 20)
            Upper = (upper[i], 255, 255)
            mask = cv2.inRange(hsv, Lower, Upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts[i] = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts[i] = imutils.grab_contours(cnts[i])
            
            if len(cnts[i])>0: ## if there is a ball of color with index i
                if checked[i]==0: ## if the ball have never been seen before
                    if rospy.get_param("state")!=4:
                        rospy.set_param("state",4)
                    active=1
                    break
                else:
                    seen[i]=1

        # only proceed if at least one contour was found
        if active:

            temp=enclosing_circle(cnts[i]) ## find the contour center and radius
            x=temp[0] ## x-coordinate
            y=temp[1] ## y-coordinate
            radius=temp[2] ## save the radius
            center=temp[3] ## coordinates of the center
            caption(image_np,x,y,radius,center,"???") ## put a caption on the circle
            # if the radius is between a minimum and and acceptable one move the robot
            # towards the ball
            if radius > 10 and radius <= max_rad:
					index=0 # a new observation of the ball
					# if the state is not 'play' set it to 'play'
					if not rospy.get_param('/state')==2:
						rospy.set_param('/state',2)
            else: # in case the robot is close enough save the ball in the server and revert to
                  # the state the robot was in before it saw the ball
                pass

        else: # if a new ball is not observed
            for i in range(len(seen)): ## put a circle around the ones that have been seen
                if seen[i]:
                    temp=enclosing_circle(cnts[i]) ## find the contour center and radius
                    x=temp[0] ## x-coordinate
                    y=temp[1] ## y-coordinate
                    radius=temp[2] ## save the radius
                    center=temp[3] ## coordinates of the center
                    if checked[i]:
                        name_i=name[i]
                    else:
                        name_i="???"
                    caption(image_np,x,y,radius,center,name) ## put a caption on the circle
                    
						
        # Show the image to screen
        cv2.imshow('window', image_np)
        cv2.waitKey(2)
	

        # self.subscriber.unregister()


## MAIN

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

## INITIALIZATION

if __name__ == '__main__':
    main(sys.argv)
