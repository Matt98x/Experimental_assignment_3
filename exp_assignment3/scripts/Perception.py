#!/usr/bin/env python

## @file robot_following.py
# @brief Perception algorithm which handles the ball tracking and the obstacle avoidance while tracking
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
threshold = 10
## Dimension of the proximity radius
max_rad=180
## robot angle
rob_angle=0
## robot vel
rob_vel=0
## maximum and minimum angle of the scan
max_ang=0
min_ang=0
# scan data
scans=[]
# minimum distance from obstacles
value_min=0.30
# robot position
rob_x=0
rob_y=0
rob_theta=0

# memorize previous state
prev_state=0

## ROOMS PARAMETERS
names=["Entrance","Living_room","Closet","Kitchen","Bathroom","Bedroom"] ## rooms name
lower=[100,50,0,25,125,0] ## lower value in first position of hsv
upper=[130,70,5,35,150,5] ## upper value in first position of hsv
checked=[0]*len(names) ## boolean of whether the ball have already been seen
## Track activate variable
active=0
## Seen balls
seen=[0]*len(checked)
## Contours
cnts=[None]*len(seen)

## GAINS
gain1=0.003
gain2=0.042
gain3=0.0035
gain4=0.04



## CALLBACKS AND FUNCTIONS

## callback function to update the robot position and orientation
def rob_callback(ros_data):
    global rob_angle,rob_vel,rob_x,rob_y,rob_theta
    temp=ros_data.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion ([temp.x, temp.y, temp.z, temp.w])
    rob_angle=yaw
    rob_vel=ros_data.twist.twist.linear.x
    rob_x=ros_data.pose.pose.position.x
    rob_y=ros_data.pose.pose.position.y
    (a,b,rob_theta)=euler_from_quaternion([ros_data.pose.pose.orientation.x,ros_data.pose.pose.orientation.y,ros_data.pose.pose.orientation.z,ros_data.pose.pose.orientation.w])


## callback function to read the laser scan
def laser_callback(ros_data):
    global max_ang,min_ang,scans
    max_ang=ros_data.angle_min
    min_ang=ros_data.angle_max
    scans=ros_data.ranges
    

## find the enclosing circle of the ball in the image
def enclosing_circle(cnts):
    # find the largest contosur in the mask, then use
    # it to compute the minimum enclosing circle and
    # centroid
    c = max(cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    return [x,y,radius,center]

## Put a caption and an enclosing circle around a perceived ball
def caption(image_np,x,y,radius,center,name):
    # draw the circle and centroid on the frame,
    # then update the list of tracked points
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.circle(image_np, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
    cv2.circle(image_np, center, 5, (0, 0, 255), -1) 
    cv2.putText(image_np,name,(int(x),int(y)), font, 1,(255,255,255),2)   

## action to take to avoid collisions
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''
    p=0.2
    if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        linear_x = 0.6
        angular_z = 0
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        linear_x = 0
        angular_z = -p
    elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        linear_x = 0
        angular_z = -p
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        linear_x = 0
        angular_z = p
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        linear_x = 0
        angular_z = -p
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        linear_x = 0
        angular_z = p
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        linear_x = 0
        angular_z = -p
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        linear_x = 0
        angular_z = -p
    else:
        state_description = 'unknown case'
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    return msg

## obstacles region declaration to obstacle avoidance
def clbk_laser(scan,k):
    regions = {
        'left':  min(min(scan[0:143]), k),
        'fleft': min(min(scan[144:287]), k),
        'front':  min(min(scan[288:431]), k),
        'fright':  min(min(scan[432:575]), k),
        'right':   min(min(scan[576:713]), k),
    }

    return take_action(regions)


## CLASS

## Image feature class to handle feature identification inside the image sent by the camera
class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel",
                                         Twist, queue_size=1)
        # subscribed Topic
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",CompressedImage, self.callback,queue_size=1)
        
		

    def callback(self, ros_data):
        global threshold, seen, checked, active, scans, prev_state
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
        active=0

        ## Check all colors to see if a ball is in the image
        for i in range(len(names)):
            if i==len(names)-1:
                Lower = (lower[i], 0, 0) # declare the minimum of the hsv range for the color
                Upper = (upper[i], 50, 50) # declare the maximum of the hsv range for the color
            else:
                Lower = (lower[i], 50, 50) # declare the minimum of the hsv range for the color
                Upper = (upper[i], 255, 255) # declare the maximum of the hsv range for the color
            mask = cv2.inRange(hsv, Lower, Upper) # find all pixels with color inside the range
            # do some manipulation to find the contours of the ball
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts[i] = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
            cnts[i] = imutils.grab_contours(cnts[i])

            if len(cnts[i])>0: ## if there is a ball of color with index i
                if checked[i]==0: ## if the ball have never been seen before
                ## if you are in a state where you are interested in tracking the ball
                    if rospy.get_param("state")==1 or rospy.get_param("state")==3:
                        prev_state=rospy.get_param("state")
                        if prev_state==3:
                            prev_state=2
                        rospy.set_param("state",4)
                    break
            else:
                seen[i]=1
            

        # only proceed if at least one contour was found and the state is active(we have seen 
        #  a new ball)
        if rospy.get_param("state")==4 and len(cnts[i])>0:
            temp=enclosing_circle(cnts[i]) ## find the contour center and radius
            x=temp[0] ## x-coordinate
            y=temp[1] ## y-coordinate
            radius=temp[2] ## save the radius
            center=temp[3] ## coordinates of the center
            caption(image_np,x,y,radius,center,"???") ## put a caption on the circle
            
            # if the radius is between a minimum and and acceptable one move the robot
            # towards the ball
            if radius >0.01 and radius <= max_rad:
                if min(scans)>0.5:
                    # control to reach the ball(find angular and linear velocities)
                    angle1=-(x-800/2) # should put the width of the image as a variable
                    vel1=(max_rad-radius)
                    # unify the two velocities
                    ang_vel=gain1*angle1
                    lin_vel=gain3*vel1
                    # apply the velocity
                    vel=Twist()
                    #if abs(ang_vel)>threshold:
                    vel.angular.z=ang_vel
                    #else:
                    vel.linear.x=lin_vel
                    self.vel_pub.publish(vel)
                else:
                    msg=clbk_laser(scans,2)
                    self.vel_pub.publish(msg)
                
            else: # in case the robot is close enough save the ball in the server and revert to
                  # the state the robot was in before it saw the ball
                active=0
                checked[i]=1
                vel=Twist()
                rospy.set_param("/"+names[i]+"/x",rob_x)
                rospy.set_param("/"+names[i]+"/y",rob_y)
                rospy.set_param("/"+names[i]+"/theta",rob_theta)
                rospy.set_param("cplace",names[i])
                if not (names[i]==rospy.get_param("command")) and prev_state==3:
                    rospy.set_param("state",3)
                else:
                    rospy.set_param("state",prev_state)

        else: # if a new ball is not observed
                for i in range(len(seen)): ## put a circle around the ones that have been seen
                    if not cnts[i]==None:
                        if len(cnts[i]):
                            temp=enclosing_circle(cnts[i]) ## find the contour center and radius
                            x=temp[0] ## x-coordinate
                            y=temp[1] ## y-coordinate
                            radius=temp[2] ## save the radius
                            center=temp[3] ## coordinates of the center
                            if checked[i]:
                                name_i=names[i]
                            else:
                                name_i="???"
                            caption(image_np,x,y,radius,center,name_i) ## put a caption on the circle
                    
						
        # Show the image to screen
        cv2.imshow('window', image_np)
        cv2.waitKey(2)
	

        # self.subscriber.unregister()


## MAIN

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.Subscriber("/odom",Odometry,rob_callback,queue_size=1)
    rospy.Subscriber("/scan",LaserScan,laser_callback,queue_size=1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

## INITIALIZATION

if __name__ == '__main__':
    main(sys.argv)
