#!/usr/bin/env python

## @file Pet_behaviours.py
# @brief Pet state machine
#
# Details: This component simulate the behaviour state machine
#

## Libraries declaration
import rospy
import time
import math
import sys
import os
import roslaunch
import random
import smach
import smach_ros
import actionlib
import actionlib_msgs
import exp_assignment3.msg

from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Pose2D
from nav_msgs.msg import Odometry
from turtlesim.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from robot_pose_ekf.srv import GetStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalID


# action service client definition
client = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
status=0

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

def status_callback(rosdata):
    global status
    if len(rosdata.status_list)>=1:
    	status=rosdata.status_list[0].status

## Subscribers definitions
rospy.Subscriber("/odom",Odometry, angle_callback,  queue_size=1)
vel_pub = rospy.Publisher("/cmd_vel",
                                       Twist, queue_size=1) # topic to command the velocity
goal_pub= rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1) # topic to impose a goal in the map
rospy.Subscriber("/move_base/status",GoalStatusArray,status_callback,queue_size=1) # topic to find the status of the move_base server
cancel_pub= rospy.Publisher("/move_base/cancel", GoalID,queue_size=1) # topic to cancel the current objective of the move_base server

## Function to cancel the current move_base goal
def cancel():
	msg=GoalID()
	cancel_pub.publish(msg)

## Function to assign the robot position via the action server
def setPosition(x,y,theta):
	global client, goal_pub
	pose=Pose() # initialization of the pose message
	pose.position.x=x #assign the x component
	pose.position.y=y #assign the y component
	pose.position.z=0 # the z component will be switched from over to under and vice versa
	temp=quaternion_from_euler(0,0,theta) # generate a orientation quaternion from euler angles
	pose.orientation.x=temp[0]
	pose.orientation.y=temp[1]
 	pose.orientation.z=temp[2]
 	pose.orientation.w=temp[3]
	msg=PoseStamped() # create the correct format
	msg.header.frame_id="map" # impose the map as the frame id
	print(pose.position.x,pose.position.y)
	msg.pose=pose # initialize a goal object with the correct format
	goal_pub.publish(msg) # Sends the goal to the action server.
	
	#client.wait_for_result() # wait for the result to come 
			
## roam: function to simulate the roaming behaviour of normal
def roam():
	global x,y
	## Command decision
	dx=random.uniform(-7,7) # find a random point in the grid for x
	dy=random.uniform(-7,7) # find a random point in the grid for y
	dtheta=random.uniform(-math.pi,math.pi)
	setPosition(dx,dy,dtheta)  # set the new goal position

## Sleep: class that describes the Sleep state
class Sleep(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
	

    def execute(self, userdata):
				## Check if in normal or not
				cancel() # cancel previous goals
				## while not at home
				x=rospy.get_param('/home/x')
				y=rospy.get_param('/home/y')
				theta=rospy.get_param('/home/theta')
				setPosition(x,y,theta)
				while True: ## change state if not sleep
					state=rospy.get_param('/state')
					if not state==0:
						return 'outcome1'
		
## Normal: class that describes the Normal state
class Normal(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3'])

    def execute(self, userdata):
				cancel() # cancel previous goals
				var=1 # if you are roaming
				while True:
					if var==1: # if to not preempt the roam goal if the pet is going there
						## roam
						roam()
						time.sleep(10)
						var=0
					else:
						if (status==3 or status==4): # the goal was achieved
							print("a new objective")
							var=1
					## check the state
					state=rospy.get_param('/state')
					if state==0:
						return 'outcome1'
					elif state==2:
						return 'outcome2'
					elif state==4:
						return 'outcome3'

## Play: class that describes the Play state
class Play(smach.State):
	
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3'])

    def execute(self, userdata):
				cancel() # cancel previous goals
				while True:
					## check the state
					state=rospy.get_param('/state')		
					if state==0:
						return 'outcome1'
		
					if state==1:
						return 'outcome2'
					
					if state==3:
						return 'outcome3'

## Find: class that describes the Find state
class Find(smach.State):
	
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
				cancel() # cancel previous goals
				package = 'explore_lite' # package of the exploration code
				executable = 'explore' # exploration launcher
				node = roslaunch.core.Node(package, executable) # node declaration
				launch = roslaunch.scriptapi.ROSLaunch() # launcher initialization
				launch.start() # launcher start
				process = launch.launch(node) # launching the node
				while True:
					## check the state
					state=rospy.get_param('/state')
					if not state==3: # if we are not in find anymore
						process.stop() # we stop the launcher 
						if state==0: # if the robot want to go to sleep
							return 'outcome1'
			
						if state==4: # if the robot found a ball to track
							return 'outcome2'

## Track: class that describes the Track state
class Track(smach.State):
	
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
				cancel() # cancel the current objective if any are present
				while True:
					## check the state
					state=rospy.get_param('/state')		
					if state==1: # return to normal
						return 'outcome1'
		
					if state==2: # return to play
						return 'outcome2'


## Main function declaration

if __name__ == '__main__':
	
	## Init the ros node
	rospy.init_node("state_machine")
	
	
	client.wait_for_server()

	## Create a SMACH state machine
	sm_s_a = smach.StateMachine(outcomes=['outcome6'])

    ## Open the container
	with sm_s_a:

		## Add states to the container
		smach.StateMachine.add('SLEEP', Sleep(),transitions={'outcome1':'NORMAL'})
        ## Add states to the container
		smach.StateMachine.add('NORMAL', Normal(), 
								transitions={'outcome1':'SLEEP',
											'outcome2':'PLAY',
											'outcome3':'TRACK'})
		
	    ## Add states to the container
		smach.StateMachine.add('PLAY', Play(), 
								transitions={'outcome1':'SLEEP',
											'outcome2':'NORMAL',
											'outcome3':'FIND'})
	    
		smach.StateMachine.add('FIND',Find(),
 								transitions={'outcome1':'SLEEP',
										'outcome2':'TRACK'})

		smach.StateMachine.add('TRACK',Track(),
										transitions={'outcome1':'NORMAL',
										'outcome2':'PLAY'})
    ## Execute SMACH plan
	outcome = sm_s_a.execute()
