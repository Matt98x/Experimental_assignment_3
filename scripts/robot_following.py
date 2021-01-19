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
from sensor_msgs.msg import CompressedImage,JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

VERBOSE = False
## Initial angle definition
angle=0
## Threshold precision
threshold = 0.0001
## Objectives for swing routine
objectives=list([math.pi/4,0,-math.pi/4,0])
#index of the swing routine
index=0
## Dimension of the proximity radius
max_rad=125
## robot angle
rob_angle=0
## robot vel
rob_vel=0
# search activators
rotated=0
half_rot=2*math.pi
last_vel=0
last_ang=0

## callback function to update the angle position
def angle_callback(ros_data):
	global angle
	temp=list(ros_data.position)
	angle=temp[0]


def rob_callback(ros_data):
	global rob_angle,rob_vel
	temp=ros_data.pose.pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion ([temp.x, temp.y, temp.z, temp.w])
	rob_angle=yaw
	rob_vel=ros_data.twist.twist.linear.x

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
	self.par_pub = rospy.Publisher("/robot/vel_control_params",
                                       Pose2D, queue_size=1)
	self.rob_pub = rospy.Publisher("/robot/cmd_vel",
                                       Twist, queue_size=1)
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)
	self.camera_angle=rospy.Subscriber("/robot/joint_states",
                                           JointState, angle_callback,  queue_size=1)
	rospy.Subscriber("/robot/odom", Odometry, rob_callback,  queue_size=1)

    def callback(self, ros_data):

	global threshold , objectives , angle , index, rob_vel, rotated, half_rot,last_vel, last_ang
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
	# the ball is described as a green object
        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

	

        # only proceed if at least one contour was found
        if len(cnts) > 0 and angle*angle<(math.pi+0.1)*(math.pi+0.1)/4:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
	    msg=Pose2D()
	    msg.x=radius
	    msg.theta=angle
	    self.par_pub.publish(msg)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10 and radius <= max_rad:
		index=0 # a new observation of the ball
		# if the state is not 'play' set it to 'play'
		if not rospy.get_param('/state')==2:
			rospy.set_param('/state',2)
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel=-0.02*(center[0]-400) # center the ball in the image 
                self.vel_pub.publish(vel) # publish the velocity to the camera
            elif radius*radius>3*max_rad*max_rad/4 and rob_vel*rob_vel<0.01 and not index==4: # if close enough to the ball start the swing routine
		cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
		vel=-2*(angle-objectives[index]) # publish the velocity to the camera
		self.vel_pub.publish(vel) # publish the velocity to the camera
		if (angle-objectives[index])*(angle-objectives[index])<0.001 : 
			index+=1 # if sufficiently close to the objective update the index
							
	    else: # the c
		vel=-0.5*angle # center the camera to zero
		self.vel_pub.publish(vel) # publish the velocity to the camera
		    
	    last_vel=vel # last camera velocity(for when you do not see the ball)
	    last_ang=rob_angle # last robot angle
	    rotated=0 # you haven't completed a search rotation
	    half_rot=2*math.pi # you haven't comleted an half rotation
        else: # if the ball is not observed
	    if rospy.get_param('/state')==2: # and you are in play
			msg=Pose2D()
	    		msg.x=-0.5
	    		msg.theta=-0.5
	    		self.par_pub.publish(msg)
			 # search for the ball rotating on the spot
			if rotated==0: # while you have not completed a search revolution
				vel=Twist()
				vel.angular.z=np.sign(last_vel)*2
				da=np.sign(rob_angle-last_ang)*(rob_angle-last_ang) # compute the spanned angle
				last_ang=rob_angle
				half_rot=half_rot-da
				if half_rot<0 :
					rotated=1
				self.rob_pub.publish(vel)
					
			else: # if the search has failed go to Normal state
				rospy.set_param('/state',1) 
				rotated=0
	    vel=-0.5*(angle) # take the angle to 0
	    self.vel_pub.publish(vel) # publish the velocity to the camera
	    	
	# Show the image to screen
	cv2.imshow('window', image_np)
        cv2.waitKey(2)
	

        # self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
