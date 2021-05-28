#! /usr/bin/env python

## @file Pet_behaviours.py
# @brief Pet state machine
#
# Details: This component handles the user interface of the project
#

## Library declaration
import rospy
from std_srvs.srv import *
import random

## Variable definition
random_timer=0 # variable to make chronologically randomic the choice made by the robot
names=["Entrance","Living_room","Bedroom","Bathroom","Closet","Kitchen"]

## Main body of code
def main():
    global random_timer
    rospy.init_node('user_interface')
    rate = rospy.Rate(10) # to adjust the frequency of the code
    while True:
        ## Code to handle the user input
        string=raw_input("Input your command(type 'help') and press ENTER: ")
        devided=string.split(" ")
        if devided[0]=="play": ## set the play behavior
            rospy.set_param("state",2)
        elif devided[0]=="help": ## help command to display all options
            print("Type your command from this list(respecting the lower case):")
            print("'play': start the play behavior of the robot")
            print("'go to '+ 'room name': when the robot is at the user it will go to this room if it knows it, otherwise it will start the find behavior")
            print("'room_list': print the list of rooms(remember to respect the letters casing)")
        elif devided[0]=="go": ## Say the robot where to go in the play behavior
            if rospy.get_param("cplace")=="owner": ## Handle the case where the robot is at the owner(can receive the command)
                if rospy.get_param("destination")=="unknown" or not(rospy.get_param("destination")=="owner"):
                    rospy.set_param("command",devided[2])
                else: ## handles the case in which the robot is not at the owner(cannot receive the command)
                    print("Another destination have already been set, try again when the robot comes back")
            else:
                print("The robot is not at the owner, try again when it is")
        elif devided[0]=="room_list": ## Show the room list
            print("Entrance \n Living_room \n Bedroom \n Bathroom \n Closet \n Kitchen")
        else: ## Catch if the user introduced an incorrect command
            print("The input is incorrect, check the casing when trying again or type 'help' and press enter for the list of available commands")
        ## Code to handle the autonomous random decision of the robot
        chaos=random.uniform(0,100) ## base of the randomic decisions
        if random_timer==0 or counter==0:
            random_timer=random.uniform(75,120) ## to decide how frequently change the behavior as a multiple of the rate of this scripts
            counter=random_timer
        else:
            if not(rospy.get_param("state")==4 or rospy.get_param("state")==0) and chaos <15:
                rospy.set_param("state",0) ## put the robot to sleep
            elif rospy.get_param("state")==2 and rospy.get_param("cplace")=="owner" and rospy.get_param("destination")=="unknown" and chaos<25:
                var=random.randrange(len(names)) ## select one room to go to
                rospy.set_param("destination",names[var]) # set it as the destination
            elif (rospy.get_param("state")==1 or rospy.get_param("state")==0):
                rospy.set_param("state",2) ## set state to play both to activate the play behavior and to exit the sleep state
            counter-=1 ## start the counter between behaviors changes
        rate.sleep() ## sleep till the end of the rate
if __name__ == '__main__':
    main()
