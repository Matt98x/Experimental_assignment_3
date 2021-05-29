# Experimental robotic assignment 3

<!-- <a href="http://htmlpreview.github.io/?https://github.com/Matt98x/Experimental_assignment_3/blob/main/html/index.html" title="Documentation">Documentation</a>
<p align="center">
  <img src="https://github.com/Matt98x/Experimental_assignment_2/blob/main/Images/pet.PNG?raw=true "Title"">
</p>
<p align="center">
  Pet
</p> -->
Documentation might take some time to load completely

## Introduction

The aim of this assignment is to create a software architecture, which simulates a pet able to move autonomously inside a domestic environment, change its behavior and interact with a user, following its orders and exploring when asked to find unseen objectives.  

The map of the environment is not known at the start of the robot movement, which entails that the robot can move autonomously both avoiding obstacles and simultaneously mapping the places it moves into. The environment itself was provided with the requirements with a description of the elements in it, among which the rooms names and correspondent ball color and the position of the owner.
Here's a visual representation obtained from the simulation environment:  

The robot itself was created for this assignment to allow for all the required features and can be seen in the following picture:


It is a differential drive device with a camera(for the objectives detection and tracking) mounted on the head, and a range-finder(to allow the obstacles avoidance), which can be seen as the white box protruding from its chest.
The differential drive regulates the posterior wheels speed to achieve forward and backward movements and the robot turning around the vertical axis. The smaller front castor wheel allow for a stable support of the robot and the proper working of the differential drive. 

## Software Architecture, State Machine and communications

Here we show the main characteristics of the implemented system: the software architecture, the state machine and the communication methods.

### Architecture
The software architecture is built around a finite state machine which encodes all the robot behaviors and the conditions which govern the change of states. The logic being that all required features are just aspects of the states and their interaction, obtained by appropriately activating or deactivating parts of code depending on the system state.

A representation of the architecture can be given in the following image:

This component diagram cannot really encompass all the logic and interconnections, but shows the main actors of the architecture:
* The "Behavior" component consists of a python script implementing a smach finite state machine. This finite state machines contains the implementation of most states and employs other scripts to perform the operations inside the remaining behaviors.
* The "Perception" component is an always active script which handles the features related to the camera. These are: the objective recognition(recognise the colored balls inside the image), the tracking and the obstacle avoidance while tracking(these two while moving closer to the recognised ball)
* The "Explore_lite" refers to a package which is activated each time the pet is in the play state and is asked to go in a never previously seen room(and it deactivates otherwise). This package is not implemented by the author, but allow the exploration of the environment as a frontier elimination process, where the exploration is done when all the frontiers have been explored.
* The "MoveBase"  package is another external source which handles the motion of the robot to specific destinations. It can be imagined as the high-level controller and generates from the input the differential control output to achieve the objective while avoiding the obstacles and detecting whether the objective is achievable.
* The "User interface" handles both the randomic aspects of the robot(as the change of the states, and the inputs in case the user do not give one) and the interaction with the user, which can select when to enter the play state and where to go when the robot is at the user.
* The "Gazebo" environment handles the simulation of the entire system both of the robot movement and of the sensor data, which it distributes via publish/subscribe topics.

Althought they might seem disconnected, these nodes share information via a global parameter server which allow all nodes to have a singular always updated source of information.

### State Machine
The state machine is implemented as a python script implementing a smach class.
This is composed by the following 6 states:
* Sleep: this state makes the robot go from its current position to its "home" or "doghouse" situated arbitrarily at (4,1) in the bedroom. Once arrived it waits till the state is changed by the user(giving a play command) or by chance, and makes it go back to the normal state.
* Normal: which enables a randomic roaming around the house which is interrupted by a change of state or by the identification of a not previously seen ball(corresponding to a room)
* Play:
* Find:
* Track:
* Recovery:

More concisely, we can see the interaction between the states in the following diagram:


### Messages and parameters

The main parameters used for this implementation are:  
* ball/ball_description: the gazebo description of the ball
* /home x&y: coordinates of the home, location where the pet sleeps
* /human_description: gazebo description of the human model
* /robot/camera1: set of parameters of the camera sensor
* /robot/joint1_position_controller: parameters for the neck joint
* /robot/joint_state_controller: parameters for the robot controller
* /robot/robot_description: gazebo robot description
* /state: state of the pet(2 play,1 normal,0 sleep)
* /gazebo: gazebo anvironment parameters  

Regarding the messages, they will be listed as  
* std_msgs.String: used by the commander to the logic, in order to send the command for the ball
* std_msgs.Float64: used by the Perception and Behavious to control the neck joint
* geometry_msgs:Pose2D: used by Perception to send the radius and camera angle for the control in the play behaviour 
* geometry_msgs:Twist: used by Behaviours and Following to Gazebo in order to control the twist of the robot
* sensor_msgs:JointStates: Used to read the values of the robot states
* sensor_msgs:Image: Message with the image camera information
* exp_assignment.PlanningAction: message of the action server, it is used by the Pet_logic and Behaviour to use the action server of the ball and robot respectively
```sh
	.geometry_msgs/PoseStamped target_pose
	---
	---
	string stat
	geometry_msgs/Pose position
 ```

## Packages and file list
```sh
Experimental_assignment_2
   Lexp_assignment2
   |   L__ action
   |   |   L__ Planning.action
   |   L__ CMakeLists.txt
   |   L__ config
   |   |   L__ motors_config.yaml
   |   L__ launch
   |   |   L__ gazebo_world.launch
   |   L__ package.xml
   |   L__ scripts
   |   |   L__ go_to_point_action.py
   |   |   L__ go_to_point_ball.py
   |   L__ urdf
   |   |   L__ ball.gazebo
   |   |   L__ ball.xacro
   |   |   L__ human.urdf
   |   |   L__ robot.gazebo
   |   |   L__ robot.xacro
   |   L__ worlds
   |       L__ world_assignment.world
   Lpet_2
   |   L__ CMakeLists.txt
   |   L__ launch
   |   |   L__ launcher.launch
   |   L__ package.xml
   |   L__ scripts
   |       L__Command_giver.py
   |       L__Pet_logic.py
   |       L__Pet_behaviours.py
   |       L__state_outfit_simulation_node.py
   LImages
   Lhtml
   |   LSearch
   |   |   L....
   |   ...
   LReadme.md
   LReadme1.md
```


As already said, the implementation is based on two packages: exp_assignment2 and pet_2.  
The first handle the simulation of the environment and the movements of the elements in it. In particular, it contains the world, robot and ball description, with the additional control parameters and related topics.  
The script present in this package are just 2:  
* go_to_point_action.py: action server to handle the movement of the robot to a specified target
* go_to_point_ball.py: action server to move the ball to a specified target
Going over to the pet_2 package, this handles the pet from perception to behaviours, plus the ball movement:  
* Command_giver.py: randomically generate command for the ball to follow
* Pet_logic: receives the commands from the command_giver and convert them to movements of the ball, apart from changing the state from play and normal to sleep and sleep to normal.
* Pet_behaviours.py: is the implementation of the finite state machine, or at least, the entirety of the sleep and normal phase and the control part of the play state
* robot_following.py: implement the perception part of the robot(in particular the vision), and handles the control of the neck joint, the swing routine and the switch from normal to play and vice versa

This were the scripts which are at the core of this implementation, but, at the side, there are other scripts.  
Starting from exp_assignment2:
* gazebo_world.launch: inside the "launch" folder: handles the definition of the simulation environment and of the elements inside it, and the controller for them.
* The urdf folder contains the xacro and gazebo description of the ball and robot
* the config folder containing the motor_config.yaml, with the controller description
* Planning.action is the message of the action server
* Finally the world folder contains the world description  

## Installation and running procedure

### Installation

* Download the package from the github repository
* Set the package in the src folder of the catkin workspace
```sh
	 git clone https://github.com/Matt98x/Experimental_assignment_2.git
 ```
* Go to the main folder of the catkin workspace and launch
```sh
	 source /devel/setup.bash
	.catkin_make
 ```
### Running

* After the compiler has completed its task we can use the same shell to launch the simulation environment.
```sh
	roslauch exp_assignment2 gazebo_world.launch
 ```
* On a differrent shell we can launch the pet control
```sh
	roslauch pet_2 launcher.launch
 ```
* Now the implementation is up and running
* One can observe the world and ball behaviour in the first shell, on the other there shell there is the robot states, and whether the target is achieved

### User commands

While there is the Command_giver to generate random commands, a human user can interface with the implementation, giving command using the following command.  
* To write a command:
```sh
	rostopic pub /commander std_msgs/String "data: ''" 
 ```
where, in place of '', you can put any commands as presented before.  
* Moreover, one can set the state(play(2),normal(1) and sleep(0)), writing:
```sh
	rosparam /state state_code 
 ```
state_code is to be substitute with one of the integer code stated


## Working assumptions

The working assumptions will be discussed as the following list:
* The robot, simulating a pet, interact with a human with a ball moving in the environment and moves in a 2D surface in a simulation environment.
* Both the robot targets and its positions belongs exclusively to the map(16 by 16 grid with center in (0,0))representing the 2D environment.
* The robot has 3 main states:
	- Play
	- Normal
	- Sleep
* The logic receive forms in strings with possible form:
	- "play"
	- "hide"	
	- "go to x1 y1" (equivalent to voice command)
	- "point to x1 y1" (equivalent to pointing commands)
* Once logic receives the message it applies it to the ball
* if the command is "play", the ball is positioned above ground, if "hide", it is positioned below the ground  .
* The robot activates the play mode only when the robot perceives the ball.
* When the ball is percieved the robot tries to get to it
* When achieved the "swing routine" is initialized, for which the neck is first angled to pi/4 and then too -pi/4 and then back to center
* If the ball is not in sight anymore, the robot procede to a full rotation to find if the ball is still in the environment, if not, it switch to the normal state..
* Two predifined positions inside the map are "Owner" and "Home", which cannot be changed during the execution, and can be used instead of coordinates in giving commands.

## System features and limitations

Starting from the limitations:
* The system is not scalable in the number of individually controllable robots, but if all robots have the same state, it is scalable, even if collision between robots are not handled
* It is not scalable in the number of symbolic locations
* It is not really scalable in the number of states
* Does not distinguish between the pointing action and the vocal command, since the ball act
* The robot control is not too responsive, since the control can't have a too high gain to avoid instability and the robot toppling over.
* The movement speed is high but make a trade-off for instability
* There are some problems with the target reaching when it is on the side of the robot especially when really close to the chassis(It cannot handle small curvature radii and the robot tends to run in circles around the target)
* Underline jittering in the motors, observable when control is not yet active

Going on to the features:
* The robot is controlled in almost its entirety by the pet package, which means a high scalability and modularity
* We have the robot perspective in a separate window 
* Can show the location of the robot in the map
* The robot can check the state without being stuck in an action server loop
* The implementation is stable even with random commands and long running tests


## Possible technical improvements

There are many possible technical improvements to this architecture:  
* Modify the simulation component to make it more scalable, introducing the state change from and to sleep inside the pet_package
* Improve the control, of both the camera and the robot chassis in such a way to perform both linear and angular velocities, and in a way that the robot do not topple over
* Handle the situation when the ball is close to the side of the robot, in order to have it centered inside the robot perspective
* Add a way to avoid collisions with other obstacles, with the possible introduction of a proximity sensor of some sort
* Add multiple robots to the simulation  

## Author and contacts
Matteo Palmas: matteo.palmas7gmail.com
