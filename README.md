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
* Play: Although more complex in the implementation, this states simply allow the pet to come back to the owner every time it is sent to a known or an unknown destination. Most of the logic inside it is mostly to handle the various scenarios that can arise. For example, if the objective has never been seen, the robot will explore the environment until a ball have been found thanks to the interaction between the find state and the track state.
* Find: This node use in its implementation the code from the explore-lite package. Inside the finite state machine, in fact, there is just some code to launch the external code and to shut it down when the robot decide to go to sleep or if a ball have been found, switching to the track behavior.
* Track: As the previous, this code is implemented in an external script. The Perception script is an always running process which handles the camera feed. When a new ball is recognised and the state is appropriate, the robot use the camera and the rangefinder data to approach it, while avoiding obstacles. When it is close enough the robot saves its pose to the parameter server to be later use as the objective pose when the owner ask it to go to the room associated with that ball. In the other cases, the robot will display a contour around the identified balls and the respective room name. 
* Recovery: This state, although not included in the requirements, allow the user to temporarily override the robot to a more useful position if it gets stuck in a wall. It is activated using the 2D navigation goal tool on rviz, and just guide the robot from its current position to the goal and then switch the state back to normal.

More concisely, we can see the interaction between the states in the following diagram:


### Messages and parameters

The main parameters used for this implementation are:  
* /state: state of the pet(0-sleep,1-normal,2-play,3-find,4-track,5-recovery)
* /home/{x,y,theta}: pose assigned to the place where the robot goes to sleep(defined a priori)
* /owner/{x,y,theta}: pose assigned to the place where the robot owner resides(defined a priori)
* /"room name"/{x,y,theta}: pose assigned by the robot to the ball assigned to the room with a generic "room name" after the tracking action have been completed(it is not defined at startup)
* /cplace: string indicating the current location of the robot, during movement it is "unknown"
* /destination/{x,y,theta,name}: contains the information of the destination of the movement  
* /gazebo: gazebo anvironment parameters  

Regarding the messages, they will be listed as  
* geometry_msgs:Twist: used by Behaviours and Following to Gazebo in order to control the twist of the robot
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
Final_assignment/
\u251c\u2500\u2500 Doxyfile
\u251c\u2500\u2500 Output
\u2502   \u251c\u2500\u2500 html
\u2502   \u2502   \u251c\u2500\u2500 annotated.html
\u2502   \u2502   \u251c\u2500\u2500 annotated_dup.js
\u2502   \u2502   \u251c\u2500\u2500 arrowdown.png
\u2502   \u2502   \u251c\u2500\u2500 arrowright.png
\u2502   \u2502   \u251c\u2500\u2500 bc_s.png
\u2502   \u2502   \u251c\u2500\u2500 bdwn.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Find-members.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Find.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Find.js
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Find__coll__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Find__coll__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Find__coll__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Find__inherit__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Find__inherit__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Find__inherit__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Normal-members.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Normal.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Normal.js
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Normal__coll__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Normal__coll__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Normal__coll__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Normal__inherit__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Normal__inherit__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Normal__inherit__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Play-members.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Play.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Play.js
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Play__coll__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Play__coll__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Play__coll__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Play__inherit__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Play__inherit__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Play__inherit__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Recovery-members.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Recovery.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Recovery.js
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Recovery__coll__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Recovery__coll__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Recovery__coll__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Recovery__inherit__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Recovery__inherit__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Recovery__inherit__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Sleep-members.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Sleep.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Sleep.js
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Sleep__coll__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Sleep__coll__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Sleep__coll__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Sleep__inherit__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Sleep__inherit__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Sleep__inherit__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Track-members.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Track.html
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Track.js
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Track__coll__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Track__coll__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Track__coll__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Track__inherit__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Track__inherit__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classBehaviors_1_1Track__inherit__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classPerception_1_1image__feature-members.html
\u2502   \u2502   \u251c\u2500\u2500 classPerception_1_1image__feature.html
\u2502   \u2502   \u251c\u2500\u2500 classPerception_1_1image__feature.js
\u2502   \u2502   \u251c\u2500\u2500 classSlamGMappingNodelet-members.html
\u2502   \u2502   \u251c\u2500\u2500 classSlamGMappingNodelet.html
\u2502   \u2502   \u251c\u2500\u2500 classSlamGMappingNodelet.js
\u2502   \u2502   \u251c\u2500\u2500 classSlamGMappingNodelet__coll__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classSlamGMappingNodelet__coll__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classSlamGMappingNodelet__coll__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classSlamGMappingNodelet__inherit__graph.map
\u2502   \u2502   \u251c\u2500\u2500 classSlamGMappingNodelet__inherit__graph.md5
\u2502   \u2502   \u251c\u2500\u2500 classSlamGMappingNodelet__inherit__graph.png
\u2502   \u2502   \u251c\u2500\u2500 classes.html
\u2502   \u2502   \u251c\u2500\u2500 closed.png
\u2502   \u2502   \u251c\u2500\u2500 dir_0136a534909667e9cbf2a898cb5153b9.html
\u2502   \u2502   \u251c\u2500\u2500 dir_116596e0c729dc38d37cfe0fe56b0dcf.html
\u2502   \u2502   \u251c\u2500\u2500 dir_759eff52ee33e92141c35462e7d596f0.html
\u2502   \u2502   \u251c\u2500\u2500 dir_7c9c90449ab1b5d80ba53974162999c9.html
\u2502   \u2502   \u251c\u2500\u2500 dir_994e7a8b085fdbb2ba4c7924a58d5edc.html
\u2502   \u2502   \u251c\u2500\u2500 dir_be4942ffe1a87acd682939009f47bbc5.html
\u2502   \u2502   \u251c\u2500\u2500 dir_c4e379ddd84f01aa8654d40405866908.html
\u2502   \u2502   \u251c\u2500\u2500 dir_e1ec3933232776542ad5c2f6e6441b3c.html
\u2502   \u2502   \u251c\u2500\u2500 dir_e6add6e1e4d3a6f847ec1265a5533f0d.html
\u2502   \u2502   \u251c\u2500\u2500 doc.png
\u2502   \u2502   \u251c\u2500\u2500 doxygen.css
\u2502   \u2502   \u251c\u2500\u2500 doxygen.png
\u2502   \u2502   \u251c\u2500\u2500 dynsections.js
\u2502   \u2502   \u251c\u2500\u2500 folderclosed.png
\u2502   \u2502   \u251c\u2500\u2500 folderopen.png
\u2502   \u2502   \u251c\u2500\u2500 functions.html
\u2502   \u2502   \u251c\u2500\u2500 functions_func.html
\u2502   \u2502   \u251c\u2500\u2500 graph_legend.html
\u2502   \u2502   \u251c\u2500\u2500 graph_legend.md5
\u2502   \u2502   \u251c\u2500\u2500 graph_legend.png
\u2502   \u2502   \u251c\u2500\u2500 hierarchy.html
\u2502   \u2502   \u251c\u2500\u2500 hierarchy.js
\u2502   \u2502   \u251c\u2500\u2500 index.html
\u2502   \u2502   \u251c\u2500\u2500 inherit_graph_0.map
\u2502   \u2502   \u251c\u2500\u2500 inherit_graph_0.md5
\u2502   \u2502   \u251c\u2500\u2500 inherit_graph_0.png
\u2502   \u2502   \u251c\u2500\u2500 inherit_graph_1.map
\u2502   \u2502   \u251c\u2500\u2500 inherit_graph_1.md5
\u2502   \u2502   \u251c\u2500\u2500 inherit_graph_1.png
\u2502   \u2502   \u251c\u2500\u2500 inherit_graph_2.map
\u2502   \u2502   \u251c\u2500\u2500 inherit_graph_2.md5
\u2502   \u2502   \u251c\u2500\u2500 inherit_graph_2.png
\u2502   \u2502   \u251c\u2500\u2500 inherits.html
\u2502   \u2502   \u251c\u2500\u2500 jquery.js
\u2502   \u2502   \u251c\u2500\u2500 nav_f.png
\u2502   \u2502   \u251c\u2500\u2500 nav_g.png
\u2502   \u2502   \u251c\u2500\u2500 nav_h.png
\u2502   \u2502   \u251c\u2500\u2500 navtree.css
\u2502   \u2502   \u251c\u2500\u2500 navtree.js
\u2502   \u2502   \u251c\u2500\u2500 navtreedata.js
\u2502   \u2502   \u251c\u2500\u2500 navtreeindex0.js
\u2502   \u2502   \u251c\u2500\u2500 open.png
\u2502   \u2502   \u251c\u2500\u2500 resize.js
\u2502   \u2502   \u251c\u2500\u2500 search
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_0.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_0.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_1.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_1.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_2.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_2.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_3.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_3.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_4.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_4.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_5.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_5.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_6.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_6.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_7.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 all_7.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_0.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_0.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_1.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_1.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_2.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_2.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_3.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_3.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_4.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_4.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_5.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_5.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_6.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 classes_6.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 close.png
\u2502   \u2502   \u2502   \u251c\u2500\u2500 functions_0.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 functions_0.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 mag_sel.png
\u2502   \u2502   \u2502   \u251c\u2500\u2500 nomatches.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 pages_0.html
\u2502   \u2502   \u2502   \u251c\u2500\u2500 pages_0.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 search.css
\u2502   \u2502   \u2502   \u251c\u2500\u2500 search.js
\u2502   \u2502   \u2502   \u251c\u2500\u2500 search_l.png
\u2502   \u2502   \u2502   \u251c\u2500\u2500 search_m.png
\u2502   \u2502   \u2502   \u251c\u2500\u2500 search_r.png
\u2502   \u2502   \u2502   \u2514\u2500\u2500 searchdata.js
\u2502   \u2502   \u251c\u2500\u2500 splitbar.png
\u2502   \u2502   \u251c\u2500\u2500 sync_off.png
\u2502   \u2502   \u251c\u2500\u2500 sync_on.png
\u2502   \u2502   \u251c\u2500\u2500 tab_a.png
\u2502   \u2502   \u251c\u2500\u2500 tab_b.png
\u2502   \u2502   \u251c\u2500\u2500 tab_h.png
\u2502   \u2502   \u251c\u2500\u2500 tab_s.png
\u2502   \u2502   \u2514\u2500\u2500 tabs.css
\u2502   \u2514\u2500\u2500 latex
\u2502       \u251c\u2500\u2500 Makefile
\u2502       \u251c\u2500\u2500 annotated.tex
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Find.tex
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Find__coll__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Find__coll__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Find__inherit__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Find__inherit__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Normal.tex
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Normal__coll__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Normal__coll__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Normal__inherit__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Normal__inherit__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Play.tex
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Play__coll__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Play__coll__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Play__inherit__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Play__inherit__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Recovery.tex
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Recovery__coll__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Recovery__coll__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Recovery__inherit__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Recovery__inherit__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Sleep.tex
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Sleep__coll__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Sleep__coll__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Sleep__inherit__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Sleep__inherit__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Track.tex
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Track__coll__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Track__coll__graph.pdf
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Track__inherit__graph.md5
\u2502       \u251c\u2500\u2500 classBehaviors_1_1Track__inherit__graph.pdf
\u2502       \u251c\u2500\u2500 classPerception_1_1image__feature.tex
\u2502       \u251c\u2500\u2500 classSlamGMappingNodelet.tex
\u2502       \u251c\u2500\u2500 classSlamGMappingNodelet__coll__graph.md5
\u2502       \u251c\u2500\u2500 classSlamGMappingNodelet__coll__graph.pdf
\u2502       \u251c\u2500\u2500 classSlamGMappingNodelet__inherit__graph.md5
\u2502       \u251c\u2500\u2500 classSlamGMappingNodelet__inherit__graph.pdf
\u2502       \u251c\u2500\u2500 dir_0136a534909667e9cbf2a898cb5153b9.tex
\u2502       \u251c\u2500\u2500 dir_116596e0c729dc38d37cfe0fe56b0dcf.tex
\u2502       \u251c\u2500\u2500 dir_759eff52ee33e92141c35462e7d596f0.tex
\u2502       \u251c\u2500\u2500 dir_7c9c90449ab1b5d80ba53974162999c9.tex
\u2502       \u251c\u2500\u2500 dir_994e7a8b085fdbb2ba4c7924a58d5edc.tex
\u2502       \u251c\u2500\u2500 dir_be4942ffe1a87acd682939009f47bbc5.tex
\u2502       \u251c\u2500\u2500 dir_c4e379ddd84f01aa8654d40405866908.tex
\u2502       \u251c\u2500\u2500 dir_e1ec3933232776542ad5c2f6e6441b3c.tex
\u2502       \u251c\u2500\u2500 dir_e6add6e1e4d3a6f847ec1265a5533f0d.tex
\u2502       \u251c\u2500\u2500 doxygen.sty
\u2502       \u251c\u2500\u2500 hierarchy.tex
\u2502       \u251c\u2500\u2500 index.tex
\u2502       \u2514\u2500\u2500 refman.tex
\u251c\u2500\u2500 README.md
\u251c\u2500\u2500 exp_assignment3
\u2502   \u251c\u2500\u2500 CMakeLists.txt
\u2502   \u251c\u2500\u2500 action
\u2502   \u2502   \u2514\u2500\u2500 Planning.action
\u2502   \u251c\u2500\u2500 launch
\u2502   \u2502   \u251c\u2500\u2500 General.launch
\u2502   \u2502   \u251c\u2500\u2500 gmapping.launch
\u2502   \u2502   \u251c\u2500\u2500 launcher.launch
\u2502   \u2502   \u251c\u2500\u2500 move_base.launch
\u2502   \u2502   \u251c\u2500\u2500 move_plan.launch
\u2502   \u2502   \u251c\u2500\u2500 rviz.launch
\u2502   \u2502   \u2514\u2500\u2500 simulation.launch
\u2502   \u251c\u2500\u2500 nodelet_plugins.xml
\u2502   \u251c\u2500\u2500 package.xml
\u2502   \u251c\u2500\u2500 param
\u2502   \u2502   \u251c\u2500\u2500 base_local_planner_params.yaml
\u2502   \u2502   \u251c\u2500\u2500 costmap_common_params.yaml
\u2502   \u2502   \u251c\u2500\u2500 global_costmap_params.yaml
\u2502   \u2502   \u251c\u2500\u2500 local_costmap_params.yaml
\u2502   \u2502   \u2514\u2500\u2500 move_base_params.yaml
\u2502   \u251c\u2500\u2500 rviz
\u2502   \u2502   \u2514\u2500\u2500 sim.rviz
\u2502   \u251c\u2500\u2500 scripts
\u2502   \u2502   \u251c\u2500\u2500 Behaviors.py
\u2502   \u2502   \u251c\u2500\u2500 Perception.py
\u2502   \u2502   \u251c\u2500\u2500 bug_m.py
\u2502   \u2502   \u251c\u2500\u2500 gmapping_muting.sh
\u2502   \u2502   \u251c\u2500\u2500 go_to_point_action.py
\u2502   \u2502   \u251c\u2500\u2500 go_to_point_service_m.py
\u2502   \u2502   \u251c\u2500\u2500 interface.sh
\u2502   \u2502   \u251c\u2500\u2500 user_interface.py
\u2502   \u2502   \u2514\u2500\u2500 wall_follow_service_m.py
\u2502   \u251c\u2500\u2500 src
\u2502   \u2502   \u251c\u2500\u2500 main.cpp
\u2502   \u2502   \u251c\u2500\u2500 nodelet.cpp
\u2502   \u2502   \u251c\u2500\u2500 replay.cpp
\u2502   \u2502   \u251c\u2500\u2500 slam_gmapping.cpp
\u2502   \u2502   \u2514\u2500\u2500 slam_gmapping.h
\u2502   \u251c\u2500\u2500 urdf
\u2502   \u2502   \u251c\u2500\u2500 Pet
\u2502   \u2502   \u2502   \u251c\u2500\u2500 robot.gazebo
\u2502   \u2502   \u2502   \u2514\u2500\u2500 robot.xacro
\u2502   \u2502   \u2514\u2500\u2500 human.urdf
\u2502   \u2514\u2500\u2500 worlds
\u2502       \u2514\u2500\u2500 house2.world
\u251c\u2500\u2500 m-explore
\u2502   \u251c\u2500\u2500 LICENSE
\u2502   \u251c\u2500\u2500 README.md
\u2502   \u2514\u2500\u2500 explore
\u2502       \u251c\u2500\u2500 CHANGELOG.rst
\u2502       \u251c\u2500\u2500 CMakeLists.txt
\u2502       \u251c\u2500\u2500 doc
\u2502       \u2502   \u251c\u2500\u2500 architecture.dia
\u2502       \u2502   \u251c\u2500\u2500 screenshot.png
\u2502       \u2502   \u2514\u2500\u2500 wiki_doc.txt
\u2502       \u251c\u2500\u2500 include
\u2502       \u2502   \u2514\u2500\u2500 explore
\u2502       \u2502       \u251c\u2500\u2500 costmap_client.h
\u2502       \u2502       \u251c\u2500\u2500 costmap_tools.h
\u2502       \u2502       \u251c\u2500\u2500 explore.h
\u2502       \u2502       \u2514\u2500\u2500 frontier_search.h
\u2502       \u251c\u2500\u2500 launch
\u2502       \u2502   \u251c\u2500\u2500 explore.launch
\u2502       \u2502   \u2514\u2500\u2500 explore_costmap.launch
\u2502       \u251c\u2500\u2500 package.xml
\u2502       \u2514\u2500\u2500 src
\u2502           \u251c\u2500\u2500 costmap_client.cpp
\u2502           \u251c\u2500\u2500 explore.cpp
\u2502           \u2514\u2500\u2500 frontier_search.cpp
\u251c\u2500\u2500 motion_plan
\u2502   \u251c\u2500\u2500 CMakeLists.txt
\u2502   \u251c\u2500\u2500 action
\u2502   \u2502   \u2514\u2500\u2500 Planning.action
\u2502   \u251c\u2500\u2500 config
\u2502   \u2502   \u2514\u2500\u2500 motors_config2.yaml
\u2502   \u251c\u2500\u2500 launch
\u2502   \u2502   \u2514\u2500\u2500 gazebo_arm3.launch
\u2502   \u251c\u2500\u2500 package.xml
\u2502   \u251c\u2500\u2500 scripts
\u2502   \u2502   \u251c\u2500\u2500 go_to_point.py
\u2502   \u2502   \u2514\u2500\u2500 go_to_point_action.py
\u2502   \u251c\u2500\u2500 src
\u2502   \u2502   \u2514\u2500\u2500 move_client.cpp
\u2502   \u2514\u2500\u2500 urdf
\u2502       \u251c\u2500\u2500 m2wr_arm3.gazebo
\u2502       \u251c\u2500\u2500 m2wr_arm3.xacro
\u2502       \u2514\u2500\u2500 materials.xacro
\u2514\u2500\u2500 slam_gmapping
    \u251c\u2500\u2500 README.md
    \u2514\u2500\u2500 slam_gmapping
        \u251c\u2500\u2500 CHANGELOG.rst
        \u251c\u2500\u2500 CMakeLists.txt
        \u2514\u2500\u2500 package.xml


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
