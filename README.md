ReadMe

# Self Driving Car Comps

# Overview
This repository provides  C++ plugin files, and XML model and world files to control the operation of an autonomous vehicle in a simulated environment.
This repository is submitted for partial fulfillment of the Carleton College Computer Science department capstone seminar.
This project is an extension of a previous Capstone Seminar.  The original project can be found at https://github.com/kileymaki/Self-Driving-Comps

# Current project contributors: Kirsten Baker, Tom Choi, Will Richards, Ruyi Shen, and Ben Withbroe


# Autonomous Vehicle Capabilities
	- Accurately detect and follow straight and curved lanes
	- Detect and avoid reasonably sized obstacles and straight, and slightly curved lanes
	- Get back to original lane after performing an avoidance maneuver.
	- Transition between any of these operations on a closed course indefinitely.

# Vehicle Design
High Level Controller
	-Controls the high level decision making of the vehicle.  Decides what ‘high level’ operation to perform ( avoid an obstacle, follow the road, return to lane etc.)

Low Level Controller 
	-controls the mechanical operation of the vehicle.  Once the HLC ‘decides where to go’, the LLC ensures that the car will go there.
Lidar
	-processes lidar sensor information from the cars lidar sensor to detect obstacles
Camera 
	-processes camera sensor information to detect road lanes, and generate waypoints along the road to follow.
GPS
	-gets cars position in the world

# Dependancies
Gazebo - Physics simulator supporting our model, world, and plugin files -http://gazebosim.org
	-Gazebo is dependent on the FADBAD++ and Boost libraries.
OpenCV - Camera Vision mathematics library used in image processing - http://opencv.org


# Homebrew Install Guide
//Install the simulator
(1) brew install gazebo6 (We do not guarantee support for any versions of gazebo later than gazebo 6.0)
	-this will also install fadbad++ and boost library packages

//Install dependancies
(2) brew install opencv

//Clone repo
(3) git clone https://github.com/shenruyi1994/SelfDrivingCarTeamZero.git

//Change directory to cloned repo
(4) cd SelfDrivingCarTeamZero.git

//Create build direcory
(5) mkdir build

// use cmake to generate makefile in build directory
(6) cd build && cmake .. && cd ..

//Script to easily make project
(7) ./sdcMake

// Runs an input world file in gazebo (use -u to start the simulation in a paused state)
(8) gazebo [worldfile.world] [-u]
