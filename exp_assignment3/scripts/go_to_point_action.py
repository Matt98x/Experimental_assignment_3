#!/usr/bin/env python

## @file go_to_point_action.py
# @brief Script implementing an action plan for the robot to move in a required position in the map
#
# Details: This component implements an action server which, on demand, control the robot to a specified location in the 2D map, communicating to the client the status of the operation
#

## import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import motion_plan.msg 

## robot state variables
position_ = Point()
pose_ = Pose()
yaw_ = 0
## machine state
state_ = 0
## goal
desired_position_ = Point()
desired_position_.z = 0
## parameters
yaw_precision_ = math.pi / 9 ## +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90 ## +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = 3.0
kp_d = 0.2
ub_a = 0.9
lb_a = -0.9
ub_d = 0.6

## publisher
pub = None

## action_server
act_s = None

## callbacks
def clbk_odom(msg):
	global position_
	global pose_
	global yaw_
	
	# position
	position_ = msg.pose.pose.position
	pose_ = msg.pose.pose
	
	# yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]
## change state
def change_state(state):
	global state_
	state_ = state
	print( 'State changed to [%s]' % state_)

def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle

def fix_yaw(des_pos):
	global yaw_, pub, yaw_precision_2_, state_
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = normalize_angle(desired_yaw - yaw_)

	
	twist_msg = Twist()
	if math.fabs(err_yaw) > yaw_precision_2_:
		twist_msg.angular.z = kp_a*err_yaw 
		if twist_msg.angular.z > ub_a:
			twist_msg.angular.z = ub_a
		elif twist_msg.angular.z < lb_a: 
			twist_msg.angular.z = lb_a
	
	pub.publish(twist_msg)
	
	# state change conditions
	if math.fabs(err_yaw) <= yaw_precision_2_:
		print( 'Yaw error: [%s]' % err_yaw)
		change_state(1)

## function to go straight ahead
def go_straight_ahead(des_pos):
	global yaw_, pub, yaw_precision_, state_
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = desired_yaw - yaw_
	err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
	err_yaw = normalize_angle(desired_yaw - yaw_)


	if err_pos > dist_precision_:
		twist_msg = Twist()
		twist_msg.linear.x = 0.8
		if twist_msg.linear.x > ub_d:
			twist_msg.linear.x = ub_d
		
		twist_msg.angular.z = kp_a*err_yaw
		pub.publish(twist_msg)
	else:
		print( 'Position error: [%s]' % err_pos)
		change_state(2)
	
	# state change conditions
	if math.fabs(err_yaw) > yaw_precision_:
		print( 'Yaw error: [%s]' % err_yaw)
		change_state(0)

## Function to terminate the control when target is achieved 
def done():
	twist_msg = Twist()
	twist_msg.linear.x = 0
	twist_msg.angular.z = 0
	pub.publish(twist_msg)
	
## Function to plan the control to reach the target
def planning(goal):
	
	global state_,desired_position_
	global act_s
	
	desired_position_.x = goal.target_pose.pose.position.x
	desired_position_.y = goal.target_pose.pose.position.y
	
	state_ = 0
	rate = rospy.Rate(20)
	success = True
	
	feedback = motion_plan.msg.PlanningFeedback()
	result = motion_plan.msg.PlanningResult()
	
	while not rospy.is_shutdown():
		if act_s.is_preempt_requested():
			rospy.loginfo('Goal was preempted')
			act_s.set_preempted()
			success = False
			break
		elif state_ == 0:
			feedback.stat = "Fixing the yaw"
			feedback.position = pose_
			act_s.publish_feedback(feedback)
			fix_yaw(desired_position_)
		elif state_ == 1:
			feedback.stat = "Angle aligned"
			feedback.position = pose_
			act_s.publish_feedback(feedback)
			go_straight_ahead(desired_position_)
		elif state_ == 2:
			feedback.stat = "Target reached!"
			feedback.position = pose_
			act_s.publish_feedback(feedback)
			done()
			break
		else:
			rospy.logerr('Unknown state!')
		
		rate.sleep()
	if success:
		rospy.loginfo('Goal: Succeeded!')
		act_s.set_succeeded(result)

## main function
def main():
	global pub, active_, act_s
	rospy.init_node('go_to_point')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	act_s = actionlib.SimpleActionServer('/reaching_goal', motion_plan.msg.PlanningAction, planning, auto_start = False)
	act_s.start()
	
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
	main()