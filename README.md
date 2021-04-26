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

In order to build the map of the house and make the robot navigate around it properly(avoiding abstacles) I used the Gmapping filtering-based approach and the MoveBase package of the ROS Navigation stack. Since the MoveBase implements an action server, inside the folder src of the exp-Assignment3 package I implemented the action client 'simple_navigation_goals', responsible for sending goals to the action server and therefore move the robot to the correct position. Inside the folder script of the same package I implemented the state machine 'state_machine.py', that leads the whole process. This node makes use of some data: inside the workspace the file 'param.yaml' can be found, which contains default data that will be stored in the parameter server once the program is executed. In the first x line we can find the representation I decided to use in order to associate each room with a color. If we consider the first line:

Closet: [-5.41, 2.48, 'F', red_ball]

I described each room with a list of arguments, the first two indicates the (x,y) coordinates of the room(extracted using Rviz), the third indicates if the room as already been discovered by the robot, while the last simply associate the room to a ball as required.
The last line

rooms: ['red_ball', 'green_ball', 'blue_ball', 'black_ball', 'magenta_ball', 'yellow_ball']



Inside the state machine are implemented the four abovementioned states:

* Normal State: relying on the MoveBase packages, the robot moves randomly around the house; in particular, a new target is sent everytime the robot stops moving. While moving the robot may detect a new colored object in the house. When this happens, the robot switches to the function Track. In this function the robot, after getting closer to the detected object, updates the parameter server marking as discovered the corresponding room(the third parameter in the list of the room is set to 'T'). 
* Sleep State: the robot goes to a predifined location(taken from the parameter server) and stays there for a certain time. After that goes back to Normal behaviour.
* Play State: the robot goes to the play pose waiting for a goTo command. The goTo command is simulated extracting randomly from the parameter server one of the six rooms. The robot checks if the extracted room has already been discovered. If so, goes there using the (x,y) coordinates stored in the parameter server. After this it goes back to the play pose waiting for another goTo command. On the contraty, if the room has not been discovered yet, it switches to the Find state passing as parameter the name of the room.
* Find state: the robot start moving around the house. When it detects a new ball it switches to the Track function that works as in the normal state. Then the robot checks if the just discovered room coincide with the room received by the Play state. If so, it goes back to the normal behaviour; if not, keeps moving around the house.


## Packages and file list
Inside the workspace we can find the following packages:
* exp_assignment3: contains the state machine inside the folder script, the simple_navigation_goals action client inside the folder src, the launch file simulation.launch inside the folder launch.
* planning: contains the gmapping.launch and the the move_base.launch files inside the launch folder.
* gmapping: contains the description of the robot inside the urdf folder and the code AGGIUNGO

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
* launch the simulation
  ```sh
      $ roslaunch exp_assignment3 simulation.launch 
  ```

