#!/usr/bin/env python

## @file Pet_behaviours.py
# @brief Pet state machine
#
# Details: This component simulate the behaviour state machine
#

## Libraries declaration
import rospy


from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Pose2D
from turtlesim.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from robot_pose_ekf.srv import GetStatus
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import time
import math
import sys
import random
import smach
import smach_ros
import actionlib
import actionlib.msg
import exp_assignment2.msg


## Variable definition
## global x position
x=0 
## global y position
y=0 
## global theta position
theta=0
## command received 
comm=0
## x target 
xt=0 
## y target
yt=0 
## delay timer
timer=0.5  
## background color publisher
pub=0
## background resets
resets=0
# action service client definition
client = actionlib.SimpleActionClient('/robot/reaching_goal', exp_assignment2.msg.PlanningAction)
## coordinates component of the ball
x_comp=0
y_comp=0
z_comp=1
## camera angle
radius=0
camera_angle=0
## robot angle
rob_angle=0

## Callbacks for the subscribers
def control_callback(ros_data):
	global camera_angle , radius	
	camera_angle=ros_data.theta
	radius=ros_data.x

def angle_callback(ros_data):
	global rob_angle
	temp=ros_data.pose.pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion ([temp.x, temp.y, temp.z, temp.w])
	rob_angle=yaw	

## Subscribers definitions
rospy.Subscriber("/robot/vel_control_params",
                                           Pose2D, control_callback,  queue_size=1)
rospy.Subscriber("/robot/odom",Odometry, angle_callback,  queue_size=1)
vel_pub = rospy.Publisher("/robot/cmd_vel",
                                       Twist, queue_size=1)

## Function to assign the robot position via the action server
def setPosition(x,y):
	global client
	pose=Pose() # initialization of the pose message
	pose.position.x=x #assign the x component
	pose.position.y=y #assign the y component
	pose.position.z=0 # the z component will be switched from over to under and vice versa
	msg=PoseStamped() # create the correct format
	msg.pose=pose # initialize a goal object with the correct format
	goal=exp_assignment2.msg.PlanningGoal(target_pose=msg) # initialize a goal object with the correct format
	client.send_goal(goal) # Sends the goal to the action server.
	
	#client.wait_for_result() # wait for the result to come 
			
## roam: function to simulate the roaming behaviour of normal
def roam():
	global x,y
	## Command decision
	dx=random.randint(-7,7) # find a random point in the grid for x
	dy=random.randint(-7,7) # find a random point in the grid for y 
	setPosition(dx,dy)  # set the new goal position

## Sleep: class that describes the Sleep state
class Sleep(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
	

    def execute(self, userdata):
	global x,y,client
	## Check if in normal or not
	client.cancel_goal() # cancel previous goals
	
	## while not at home
	xtar=rospy.get_param('/home/x')
	ytar=rospy.get_param('/home/y')
	setPosition(xtar,ytar)
	while True: ## change state if not sleep
		state=rospy.get_param('/state')
		if not state==3:
			return 'outcome1'
		
		
## Normal: class that describes the Normal state
class Normal(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	client.cancel_goal() # cancel previous goals
	var=1 # if you are roaming
	while True:
		
		if var==1: # if to not preempt the roam goal if the pet is going there
			## roam
			roam()
			var=0
		else:
		     if client.get_state()==3: # the goal was achieved
			var=1
		## check the state
		state=rospy.get_param('/state')
  		if state==2:
			return 'outcome1'
		elif state==3:
			return 'outcome2'

## Play: class that describes the Play state
class Play(smach.State):
	
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global camera_angle , radius, rob_angle, vel_pub,client
	client.cancel_goal() # cancel previous goals
	while True:
		
		vel=Twist()
		if radius>-0.5:
			# logic to go closer to the ball but not close enough to flip the robot
			if radius<=125:
				vel.linear.x=-0.012*(radius-125)
				if camera_angle*camera_angle>0.001:		
					vel.angular.z=2*(camera_angle)
			else:
				if radius<128:
					vel.linear.x=-0.012*(radius-128)
			
			vel_pub.publish(vel)
		## check the state
		state=rospy.get_param('/state')		
		if state==3:
			return 'outcome2'
		
		if state==1:
			return 'outcome1'


## Main function declaration

if __name__ == '__main__':
	global client,x_comp,y_comp,z_comp
	## Init the ros node
	rospy.init_node("state_machine")
	
	
	client.wait_for_server()

	

	## Create a SMACH state machine
        sm_s_a = smach.StateMachine(outcomes=['outcome4'])

        ## Open the container
        with sm_s_a:

	    ## Add states to the container
	    smach.StateMachine.add('NORMAL', Normal(), 
				          transitions={'outcome1':'PLAY', 
				                       'outcome2':'SLEEP'})
            ## Add states to the container
            smach.StateMachine.add('SLEEP', Sleep(), 
					transitions={'outcome1':'NORMAL'})

	    ## Add states to the container
	    smach.StateMachine.add('PLAY', Play(), 
				          transitions={'outcome1':'NORMAL', 
				                       'outcome2':'SLEEP'})
	    
            ## Execute SMACH plan
            outcome = sm_s_a.execute()
