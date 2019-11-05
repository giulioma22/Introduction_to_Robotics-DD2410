# Introduction to Robotics - DD2410

## Lab 2 - Inverse Kinematics

In this assignment you will program the inverse kinematic algorithm for a robot, which will move its joints so that it follows a desired 
path with the end-effector. 

It is composed of two parts:
1) A 3 Degrees-Of-Freedom (DOF) scara robot, with an inverse kinematic solution that is possible in analytic form.
2) A 7 DOF kuka robot, with the inverse kinematic solution to be implemented with iterative algorithms. 

## Lab 3 - Planning

In this assignment you’re tasked to implement a robotic planning method in order to drive a Dubin’s car, with the following dynamics:

| Dubin's car dynamics  |
| --- |
| x[t+1] = x[t] + cos(theta[t])  |
| y[t+1] = y[t] + sin(theta[t])  |
| theta[t+1] = theta[t] + tan(phi[t])  |

from an initial position (x0,y0) to a target position (xt, yt), while avoiding both collisions with obstacles and 
venturing out of bounds.

The state variables are:
- x: horizontal position
- y: vertical position
- theta: heading angle (direction of travel)
And the sole control variable is the steering angle phi ∈ [-pi/4, pi/4].

## Lab 4 - Mapping

Mapping is one of the core competencies of truly autonomous robots. Autonomous robots can use maps in a number of different ways, 
search and rescue robots can use them to make sure they search through the whole building instead of moving in between the same rooms 
over and over, autonomous cars use them in order to find a path that leads to the desired location, multicopters can use the maps to 
localize themself in order to stay in the air.

In many situations we cannot assume that the robot can be given a map in advance. Even if there are maps available, such as blueprints 
for a building, they are not always useful for the robot and might be incorrect (too old, building collapsed, etc). Therefore it is 
of great benefit if the robot can construct a map by itself from scratch, which is exactly what we will do in this assignment.

### Occupancy grid mapping
Occupancy grid mapping is one of many mapping algorithms. Here the world is represented as a grid, where each cell of the grid 
corresponds to an area in the world. The value of the cell can tell us if the area is free, occupied, unknown, or something else.

The occupancy grid is characterized by the number of cells and the resolution. More cells means that it is possible to map a larger area. 
In this assignment we will work with a 2D occupancy grid but 3D grid maps are often used as well. The resolution describes how big 
of an area each cell covers. If the resolution is 5cm then one cell in a 2D grid covers a 25cm² area. 
