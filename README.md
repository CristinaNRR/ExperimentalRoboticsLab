# Assignment 3 Experimental Robotics Lab

## About The Project

In this assignment it is required to use packages for autonomous navigation in order to make a robot navigate a house composed by 6 rooms. In each room it is located a ball with a specific color; each color is therefore associated to a different room as follow:

* Blue:Entrance
* Red:Closet
* Green:Living Room
* Yellow:Kitchen
* Magenta:Bathroom
* Black:Bedroom

The robot should then implement four behaviors:

* Normal 
* Sleep
* Play 
* Find

## Architecture and parameters

In order to build the map of the house and make the robot navigate around it properly(avoiding abstacles) I used the Gmapping filtering-based approach and the MoveBase package which is part of the ROS Navigation stack. The MoveBase package has the final purpose to make a robot navigate an environment using both a local and global path planner that can be selected by the user. The MobeBase relies on a map that can be known a-priori or can be built during the execution(our case). 
Since the MoveBase implements an action server, inside the folder src of the exp-Assignment3 package I implemented the action client 'simple_navigation_goals', responsible for sending goals to the action server and therefore move the robot to the correct position. The goals generated by the action client are expressed with respect to the frame of the map, and not the frame of the robot. 
The explore-lite package is used together with MoveBase and Gmapping to automatically explore the map when required. In order to enable and disable the functioning of the package during the code execution, I used
  ```sh
      $ self.child = subprocess.Popen(["roslaunch","explore_lite","explore.launch"])
      $	self.child.send_signal(signal.SIGINT)
  ```
This was necessary because the motion of the robot had to rely on that package only during the Find state.

Inside the folder script of the same package I implemented the state machine 'state_machine.py', that leads the whole process. This node communicates with the action client publishing target positions expressed as a list of integers(as shown inside the msg folder).  

The state machine makes use of some data: inside the workspace the file 'param.yaml' can be found, which contains default data that will be stored in the parameter server once the program is executed.

 In the first 6 lines we can find parameters to associate each room with a color. If we consider the first line:

Closet: [-5.41, 2.48, 'F', red_ball]

I described each room with a list of arguments, the first two indicates the (x,y) reference coordinates of the room(extracted using Rviz), the third indicates if the room has already been discovered by the robot, while the last simply associate the room to a ball as required.

With the following line I simly created a list with the rooms of the apartment 

rooms: ['red_ball', 'green_ball', 'blue_ball', 'black_ball', 'magenta_ball', 'yellow_ball']

In the end, I stored some default position

playPose: [-5, 8]
homePose: [-2.86, -2.95]



Inside the state machine are implemented the four abovementioned states:

* Normal State: relying on the MoveBase packages, the robot moves randomly around the house; in particular, a new target is sent everytime the robot stops moving. While moving the robot may detect a new colored object in the house. When this happens, the robot switches to the function Track. In this function the robot, after getting closer to the detected object, updates the parameter server marking as discovered the corresponding room(the third parameter in the list of the room is set to 'T'). After some time, the robot switches to the play or sleep state. 
* Sleep State: the robot goes to a predifined location(taken from the parameter server) and stays there for a certain time. After that goes back to Normal behaviour.
* Play State: the robot goes to the play pose(taken from the parameter server) waiting for a goTo command. The goTo command is simulated extracting randomly from the parameter 'rooms' in the parameter server. The robot checks if the extracted room has already been discovered. If so, goes there using the (x,y) coordinates stored in the parameter server. After this it goes back to the play pose waiting for another goTo command. On the contraty, if the room has not been discovered yet, it switches to the Find state passing as parameter the color of the ball in that room.
* Find state: the robot starts moving around the house relying on the explore-lite package. When it detects a new ball it switches to the Track function that works as in the normal state. Then the robot checks if the ball in the just detected room coincide with the ball received by the Play state. If so, it goes back to the play behaviour; if not, keeps moving around the house. After some time, if the robot has not detected the received room, goes back to the play behaviour.

In each state I used lots of 'loginfo' functions in order to be able to follow the state machine behaviour and check that everyting is working properly.

Below the state machine and architecture diagrams.

<img src="https://github.com/CristinaNRR/final_exp/blob/master/Images/state_machine_diagram.png" alt=" " width="600" height="400"/>
_Figure 1 : state machine diagram_

<img src="https://github.com/CristinaNRR/final_exp/blob/master/Images/Architecture_diagram.png" alt=" " width="600" height="400"/>
_Figure 2 : software architecture_

## Packages and file list
Inside the workspace we can find the following packages:
* exp_assignment3: contains the state machine inside the folder script, the simple_navigation_goals action client inside the folder src, the launch file simulation.launch inside the folder launch(which is used to launch the whole simulation, opening both Gazebo and Rviz).
* planning: contains the gmapping.launch and the the move_base.launch files inside the launch folder.
* gmapping: contains the description of the robot inside the urdf folder and the code used by the gmapping algorithm.
* explore: contains the explore.launch file that activates the explore-lite package.
Inside the doxygen folder we can find the code documentation of the project.

## Installation and running procedure
* clone the repository inside home
  ```sh
      $ git clone https://github.com/CristinaNRR/final_exp.git
  ```
* build and source the workspace
  ```sh
      $ cd final_exp
      $ catkin_make
      $ source devel/setup.bash
  ```
* launch the simulation and store the parameter in the parameter server
  ```sh
      $ roslaunch exp_assignment3 simulation.launch 
      $ rosparam load param.yaml
  ```

Looking at the terminal we can follow all the steps performed by the state machine including its change of states.

## System features and possible tecnical improvements

To test and correct the robot behaviour I added print functions in the code. This is also useful in order to follow the robot behavior during the program execution.
During the whole execution I decided to publish new target positions to the moveBase action client everytime the robot stops moving. This is the easiest solution I found to be sure that the state machine is 'syncronized' with the motion of the robot. 
While the robot is moving to some position guided by the moveBase package it may happen that it detects a colored ball in the environment: the robot will now start moving towards the ball simply publishing new commands on the cmd_vel topic. The robot will stop tracking the ball when the velocity published on cmd_vel is almost 0 or a predefined number of iteration has been reached(this last option was needed because sometimes it happens that the robot is not able to actually reach the ball because it gets stuck in some wall). Once the robot stops tracking the ball, it will continue following its previous goal relying again on the commands published by moveBase.

Some possible improvements:
* Handle the fact that sometimes the robot is required to move to a position which is outside the walls of the house.
* In order to execute the transition between normal and play state I simply introduced a counter in the normal state. Once the counter reaches a predefined value the system switches to the play state. One improvement might be to receive the play command from a stand alone node.
* As said before, the reference positions of each room, have been stored manually before the execution inside the parameter server. However, the positions will be unknown to the robot until it actually explore the rooms; only in that moment the reference positions will be marked as 'T' and therefore the robot will be able to use them. 
One improvement might be to obtain the positions at runtime once the robot gets close to the ball.

Regarding the last point, I decided to store myself the reference positions of the rooms because, as said before, I noticed that in some rare  case when the robot detects a new ball it is not always able to actually gets close to it since it may get stuck in some walls between its position and the ball. This of course may happen since in the ball tracking the robot is not relying on the moveBase package to guide its motion. 

## Author and contacts
Cristina Naso Rappis
mail: cri.tennis97@gmail.com


