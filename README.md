# Adaptive Goal Management #

### Description ###

This repository is the documentation, examples, issue tracking for a MVP app I created called Adaptive Goal Management.

Adaptive Goal Managment is an app which acts as a robot fleet manager based upon an adaptive goal execution system for robots and drones which are connected to the internet.  

It is free to test and experiment with your robots and I am currently building out examples using ROS.

### Where to find the app and documentation ###

- [Adaptive Goal Management Website](https://adaptive-goal-management.herokuapp.com/): This is the website of the app. 
- [Documentation describing what it does and how it works](https://github.com/mukmalone/AdpativeGoalManagement/blob/master/documentation/Adaptive%20Goal%20Management-Overview.pdf): This is an overview of how the app works using a manufacturing example.
- [API documentation](https://github.com/mukmalone/AdpativeGoalManagement/blob/master/documentation/API_Documentation.md): This is the API documentation and the ROS service documentation which connects to the app.

### Introduction ###
[![IMAGE ALT TEXT](http://img.youtube.com/vi/hbpWmQUFGMc/0.jpg)](https://youtu.be/hbpWmQUFGMc "AGM Quick Start")

### Quick start guide ###
[![IMAGE ALT TEXT](http://img.youtube.com/vi/ex2v6yrXj6A/0.jpg)](https://youtu.be/ex2v6yrXj6A "AGM Quick Start")

### Examples ###
In the examples folder there are examples of robots based upon ROS and simulations using the application.

#### [MIR Robot Example](https://github.com/mukmalone/AdpativeGoalManagement/tree/master/examples/mir_robot) ####

- Step 1: Clone the project to your catkin workspace
- Step 2: `catkin_make`
- Step 3: Configure your AGM workspace with one worker (see [Quickstart](https://www.youtube.com/watch?v=ex2v6yrXj6A&feature=youtu.be))
- Step 4: in `/mir_agm/launch/mir_agm.launch` modify the last node launched to have the arg equal to your Worker key:
- > `<node name="agm_worker_node" pkg="mir_agm" type="agm_worker_node" args="YOUR_WORKER_KEY_HERE"/>`
- Step 5: `roslaunch mir_agm mir_agm.launch`
- Step 6: Activate routings
