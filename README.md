# Experimental Robotics Laboratory - Assignment 2

## Introduction
This repository contains the second assignment for the University of Genoa's experimental robotics laboratory course 2020. You can use it to experiment with ros. Because it merely provides simulations, no specific hardware is required.

<!-- ABOUT THE PROJECT -->
## About The Project

The goal of this project is to become familiar with using gazebo simulations, which include plugins like as sensors and controllers. In a simulated environment, a wheeled robot moves around with a ball model that moves about randomly. On top of the robot's head sits a simulated camera. When the robot notices the ball, it immediately begins to follow it. When the robot gets close enough to the ball and it isn't moving, it looks to the left, then to the right, and finally to the front. When the robot loses the ball, it resumes its random movement. The robot goes to a gazebo to sleep for an unknown amount of time at random times.

### Built With

* [ROS noetic](http://wiki.ros.org/noetic/Installation)
* [Python3](https://www.python.org/downloads/)
* [Smach](http://wiki.ros.org/smach)
* [Gazebo](http://gazebosim.org/)

## Software architecture and state diagram
### Architecture

![Assignment 2 - architecture](https://user-images.githubusercontent.com/25705086/189081277-aa02c7d4-1c4c-4f83-9b71-11b2f4ae9cc6.jpg)

#### Components
* Human Interaction Generator
* Behaviour Controller
* Motion Controller
* Ball Tracking 

#### Description
The basic architecture is made up of four components, three of which are connected to robot control (one for behavior and two for map movement), and the fourth represents the user who orders the ball (move in a certain point or go underground).

The **Human Interaction** The component uses a SimpleActionClient to provide the ball goal positions and blocks until the goal is reached, then sleeps for a random number of seconds between 7 and 10, keeping the ball motionless.
There are two options: sending the ball underground or sending a random goal position over the ground (always in the same position [0 0 -1]). This decision is done at random, and the ball has a 25% chance of disappearing beneath the ground after each cycle.

The **Behaviour Controller** The component contains a finite state machine, which is responsible for changing the behavior of the robot. This exposes the topic's state each time the topic  changes, allowing other components to change their behavior accordingly. The three actions are normal (first action), sleep, and play. More details can be found in the State Machines section. * ball_detected * Subscribe the topic to switch from normal to play state, *  home_reached * Subscribe to the  topic to switch between sleep and normal state.

The **Movtion Controller** When the behavior is set to normal or sleep, the component handles the robot's motion. Subscribe to the  topic * /Behavior * to get the current status.  Also, instantiate a SimpleActionClient that tells the Action Server * /Go to point robot * to send the target position that the robot will reach. Under normal conditions, it selects a random position in the environment to get a random number of x and y positions from 8 to 8, which is the maximum dimension of the simulated map. Then send the destination to the server and wait for it to reach it. If the state changes when the target has not yet been reached, use the * /send_goal * function callback to cancel the current target and allow the robot to change its behavior accordingly.
In sleep mode, the motion controller sends the home position (obtained from the Ros parameter) to the action server and waits. When the goal is achieved, it will be published in the */home_reached * topic to warn the behavior controller that the robot is at home.

The **Ball tracking** This component implements the openCv algorithm to detect the ball (more specifically the color of the ball) and allow the robot to track the ball as the actual movement is being played. Subscribe to the robot camera topic (* / robot / camera1 / image_raw / compressed *) and use the OpenCv library in the subscriber callback to detect the ball in your environment. As soon as a ball is detected, a * / ball_detected * message is sent to the action controller. Then, when the state transitions to the play state, the speed will be published in the * / robot / cmd * topic, allowing the robot to track the ball. When the ball stops,  the robot stops,  stops tracking the ball, and publishes commands in the * / robot / join_position_controller / command * topic to move the robot's head pivot to the right, left, and back. It is in the starting position. After finishing, he returns to chase the ball and follows it until the condition changes or the ball stops again.

#### Action Servers
* Go to point robot
* Go to point ball

The **Go to point robot** Is connected to the movement controller. The movement controller implements the SimpleActionClient instance as described above. The robot SimpleActionServer gets the goal from the motion controller, publishes the velocity in the topic * / robot / cmd_vel *, and provides feedback until the goal is reached. It has three states. One is to fix the yaw of the robot toward the target, one is to go straight, and the last state is to stop when the target is reached.

The **Go to point ball** Very similar to a robot, but  controls the movement of the ball and posts to the topic * / ball / cmd_vel *. It consists only of linear velocity control along x, y, z, so in this case there is no yaw control. 
 There are only two states. One is to move to the target and the other is to stop when the target is reached.

#### Ros Parameters
* home_x &rarr; home x position on the map
* home_y &rarr; home y position on the map

#### Ros Topics 
List of ros topics that are directly related to the code you are developing:

* /behaviour &rarr; topic on which the current state is published by the behaviour controller
* /ball_detected &rarr; topic on which it is published when the ball is detected or not using a Bool
* /home_reached &rarr; topic on which it is published when the home is reached or not during the Sleep behaviour using a Bool
* /robot/joint_position_controller/command &rarr; topic used to move the robot head joint
* /robot/camera1/image_raw/compressed &rarr; (/robot/camera in the diagram) topic used to retrieve the images from the robot camera 
* /robot/cmd_vel &rarr; topic used to publish velocities to the robot by the ball tracking component and the Go to point robot Action Server
* /robot/odom &rarr; topic used to get the robot odometry in the Go to point robot Action Server
* /robot/reaching_goal &rarr; family of topics of the action server for the robot
* /ball/cmd_vel &rarr; topic used to publish velocities to the ball by the Go to point ball Action Server
* /ball/reaching_goal &rarr; family of topics of the action server for the ball


### State Machine
This is the state machine inside the Behaviour Controller component


The **Normal** The behavior consists of randomly moving  around the map. When the ball is detected by the camera, the ball switches to play mode. Otherwise, it may randomly switch to sleep after at least 30 seconds of normal operation.

The **Sleep** The action is to move to the starting position and stay there for a while. The transition to the normal state occurs after a random time  (20 - 40 seconds) that begins after the robot reaches its home position.

In the **Play** In operation, the robot simply chases the ball. When the ball stops (and therefore the robot also stops), move the head 45 degrees to one side, wait a few seconds and then the other side, wait a few seconds and then return to the zero position (this is managed by the ball_following node). ). When the ball is no longer detected, the counter starts and returns to normal when the ball reaches 15 seconds without being displayed again. This time  is about the same as the time it takes for the robot  to rotate 360 degrees and was chosen to be recognized again if the ball is not underground.

## Contents of the repository
Here the content of the folders contained in this repository is explained
### Action
Contains the definition of a custom action message
### Config
Contains the yaml configuration file for the joint_position_controller and joint_state_controller, managed by the ros_control plugin
### Launch
Contains two launch files. One (*gazebo_world.launch*) is for showing on gazebo the simulated world and spawning  the human, the robot, the ball and their relative action servers and joint controller.
The other one is for the behaviour architecture that manages the robot and ball movements.
### Scripts
Contains the two action servers python files: *go_to_point_ball.py* and *go_to_point_robot.py*
### Src
Contains the four python files (the components) of the main architecture: *human_interaction.py*, *behaviour_controller.py*, *movement_controller.py* and *ball_following.py*
### Urdf
Contains the descriptions of the robot model, the ball and their relative gazebo files, and the description of the human. The description of the robot has been modified to include two new links and corresponding joints, a fixed one for the neck and a revolute for the head. Moreover a transmission motor has been included to make the robot head position controllable using the ros_control plugin.
### Worlds
in the worlds folder there is the description of the simulation world that will be loaded on gazebo.


## Installation and running procedure
The first thing to do, after having cloned the repository in the Ros workspace, is to build the package, using the following command in the workspace:
    
```console
catkin_make
```
In order to run the system, you have to launch the two following launch files in this order, the first one loads the gazebo world and runs the action servers, the second one runs the rest of the architecture:

```console
roslaunch exp_assignment2 gazebo_world.launch
roslaunch exp_assignment2 behaviour_architecture.launch 
```
You need to have the ros_control and gazebo ros_control related packages installed.

## System’s features
The principal way used to stress the system is the complete randomness of the elements of the architecture: the motion of the ball is managed by the human node using random goal positions and random wait times between a movement and another. Moreover the choice of givinig a goal position over or under the ground is also random. The robot in the Normal state moves randomly on the map and always tries to detect the ball, so its behaviour can change at any time. When in the Play state the robot always follows the ball until it stops or it disappears, so it is bound to the randomness of the ball motions. The transition between Normal and Sleep behaviour is also random and it is allowed after 30 seconds of Normal behaviour, in order to make it less probable than the Play one which is more insteresting in testing the architecture robustness.\
One of the system features is the fact that when in the Normal state, the robot will always be ready to cancel the current goal and transition to the required state (Play or Sleep), thanks to the capabilities of the Action ServerClient system and the feedback messages.\
The system has been tested during various sessions, with one of them in particular that lasted two hours straight, and no major issues has been found, apart for one particular case which will be explained in the next paragraph.

## System’s limitations
The main problem I found in the early test sessions is the fact that the initial ball velocity defined in the Go-to-Point Ball node is too high. This led to two main issues. The first was the fact that it was difficult for the robot to track the ball correctly, and it was common to lose track of the ball  when  moving perpendicular to the  robot in play. Second, when the robot was chasing the ball,  the ball started to move directly against the robot's direction of movement, but because the indicated speed was opposite to the previous direction, the ball rolled and further. The fact that it can no longer be moved. The movement of things was too high.\
These issues were resolved by reducing the maximum ball speed on the action server and slightly increasing the speed of the robot while tracking the ball. In my tests, the robot never  lost control of the ball. The fall occurred again at the end of the two-hour session. This is probably because  tracking became active  when the ball was very close to the robot's camera after moving its head. In general, I've seen the ball go straight toward the robot at full speed several times in the same test and  other test sessions, but I conclude that it's a very special case because the fall never happened again. I did.\
Another limitation I've found is that  after  the robot stops playing and performs a headshake, the headshake is performed twice in a row because the ball is still in front of it. This can be resolved by slightly changing the ball's downtime and head movement length, but it's a very rare issue and doesn't affect the overall functionality of the architecture, so leave it alone. recommend to.

## Possible technical improvements
These are some possible technical improvements that can be made to make the system more realistic and powerful.
  - also sets the desired direction of the robot with normal and sleeping movements.
  - Avoid sending the ball to a human position.
  -  Add a more realistic model of a pet robot (currently the neck and head are just cylinders and boxes).
  - Add an obstacle avoidance system to the robot.
  - Remove the 30-second constraint set as needed to shift from normal operation to sleep operation and further increase randomness.




## Rqt_graph
### Main Architecture and Gazebo Simulation


## Author
Mohammed Alaaeldin Youssef Mahmoud\
E-mail: mohammed.eldin.88@hotmail.com\
ID: 4844271

