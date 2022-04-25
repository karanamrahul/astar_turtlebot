# astar_turtlebot
This project contains A* path planning algorithm implementation for Turtlebot3.


TurtleBot3 planning using A-Star using Differential constraints


### Obstacle map 
<p align="center">
<img src="https://github.com/karanamrahul/astar_turtlebot/blob/main/astar_differential/results/obstacle_map.png"/>
</p>


## A-star Output
<p align="center">
<img src="https://github.com/karanamrahul/astar_turtlebot/blob/main/astar_differential/results/output1.png"/>
</p>

## Results 



### Authors
Rahul Karanam
Harika Pendli


### Introduction to the Project
In this project, the A-star motion planning algorithm was used on ROS Turtlebot 3 to navigate in a configuration space consisting of static obstacles.



### Software Required
For this project you will need to install the rospy, numpy, heapq(Priority queue for A*), matplotlib and gazebo to run the simulations.


### Simulation platforms used
For the simulation we used the gazebo and turtlebot3 package. The world file is located in the world folder and defines the setup of the gazebo environment.


### Instructions for running the code
For running the code please follow the detailed instructions given below.
First we create a catkin workspace for our project

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

After creating your catkin workspace change your directory to src and clone this repository

```
cd ~/catkin_ws/src
git clone --recursive https://github.com/karanamrahul/astar_turtlebot.git
cd ../
catkin_make
```

After cloning the repository lets create an executable for our .py file that contains the code to run our program.

```"https://github.com/karanamrahul/astar_turtlebot/blob/main/astar_differential/results/obstacle_map.png
cd ~/catkin_ws/src/astar_turtlebot/src/scripts
chmod +x astar_tb3.py
catkin_make
```

Once all the above steps have been performed lets source our catkin workspace and then run the program

```
source ./devel/setup.bash
roslaunch astar_turtlebot demo.launch x_init:=8 y_init:=5 theta_init:=0 x_final:=7 y_final:=7 rpm1:=15 rpm2:=10
```


```
x coordinate for the start node(in meters"https://github.com/karanamrahul/astar_turtlebot/blob/main/astar_differential/results/obstacle_map.png):
y coordinate for the start node(in meters):
orientation for the start node(in radians):
x-coordinate of the goal node(in meters):
y-coordinate of the goal node(in meters):
Enter the first value of RPM:
Enter the second value of RPM:
```

After entering all these values in the terminal, the A-star algorithm finds the optimum path between the entered start node and goal node. Then the "dvx, dvy, dw" values, which are the velocities in x-direction and y-direction and angular velocity along z-axis are published on the ROS Topic of the Turtlebot to move it from one point to another point.

### A-star Turtlebot 3 Output
<p align="center">
<img src="https://github.com/karanamrahul/astar_turtlebot/blob/main/astar_differential/results/path_2.gif"/>
</p>


### Videos:
	
Drive Link : https://drive.google.com/drive/folders/1selsuN77piMY9i_CcUnBPz3N9GBXBf1B?usp=sharing
