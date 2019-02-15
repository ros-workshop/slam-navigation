SLAM & Navigation
==================
Material for day 2 of the ROS Workshop.  

TODO: add Youtube video

Introduction
-------------

### Simultaneous Localisation and Mapping (SLAM)
For a mobile robot to navigate through its environment, it requires both 1) a map and 2) knowledge of where it is in that map. This is the "Simultaneous Localisation and Mapping" or SLAM problem, which is a fundamental problem in robotics and the focus of considerable research over the last few decades.
* Spend a few minutes reading about the SLAM problem on Wikipedia [here](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping).  
* The SLAM algorithm used in this workshop is a Rao-Blackwellized Particle Filter ([this tutorial](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/rbpf-slam-tutorial-2007.pdf) has some more information on RBPFs).  
* The ROS implementation used here is called [GMapping](http://wiki.ros.org/gmapping), it was open sourced by Grisetti et al [here](https://openslam-org.github.io/gmapping.html), while [this paper](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf) describes the algorithm in detail. 

### Navigation
* 

Instructions
----------------

### Overview
This session will build on the ROS packages Clearpath makes available for their [Husky robot base](http://wiki.ros.org/Robots/Husky).  You will install and configure the gmapping SLAM package, along with the Move_base navigation package. 

ros-kinetic-husky-simulator

### Bootstrap Catkin Workspace

Create a new Catkin workspace ```workshop_ws``` and Git clone this repository into the folder ```workshop_ws/src```.

<details><summary>Click here to cheat!</summary>

```
# Install wstool and rosdep.
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep

# Create a new workspace in 'workshop_ws'
mkdir workshop_ws && cd workshop_ws
wstool init src

# Git clone repo
git -C ./src clone -b master https://github.com/ros-workshop/slam-navigation.git

#install rosdeps

ros-kinetic-husky-simulator

```
</details>


### GMapping for SLAM

http://wiki.ros.org/gmapping



### Move_base for Navigation


### Stretch Goals
* **Try on a real robot:** 
  * **Motivation:** simulated robots often miss some of the subtleties of real robots   
  * **Goal:** configure a [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) to navigate around the lab
  * **Instructions:**
    * There are limited TurtleBot3s available, please demonstrate navigation in Gazebo first
    * Follow the [instructions here](http://emanual.robotis.com/docs/en/platform/turtlebot3/navigation)
* **Geofencing your robot:** 
  * **Motivation:** we want to annotate the map to keep the robot in a particular area 
  * **Goal:** save the map to disk, edit it, and then relocalising and navigate in it
  * **Instructions:**
    * Build a complete map of the environment using ```gmapping```
    * Save the gridmap to disk and shut down ```gmapping```
    * Edit the gridmap (e.g. GIMP) to add some virtual "fences" 
    * Load the map and use the ```amcl``` package to relocalise the robot
    * Show that ```move_base``` can navigate without crossing your virtual fences 
* **Large-scale SLAM:** 
  * **Motivation:** The RBPF algorithm used in ```gmapping``` does not scale well
  * **Goal:** test Cartographer, a modern pose-graph based SLAM implementation 
  * **Instructions:**
    * Clone and build Google's Cartographer by following the instructions [here](https://google-cartographer-ros.readthedocs.io/en/latest/)
    * Remap

### Questions
* What does the tf tree and node graphs look like while navigating?
* How do you remap a topic name when starting a node in a lanch file?
