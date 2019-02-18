SLAM & Navigation
==================
Material for day 2 of the ROS Workshop.  

Goal: At the end of this session you should have a simulated robot navigating to user-selected waypoints. Random Youtube video:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=WmGVRX2r8WY
" target="_blank"><img src="http://img.youtube.com/vi/WmGVRX2r8WY/0.jpg" 
alt="Video" width="240" height="180" border="10" /></a>


Introduction
-------------

### Simultaneous Localisation and Mapping (SLAM)
For a mobile robot to navigate through its environment, it requires both 1) a map of its environment, and 2) knowledge of where it is in that map. This is the "Simultaneous Localisation and Mapping" or SLAM problem, which is a fundamental problem in robotics and the focus of considerable research over the last few decades.
* Spend a few minutes reading about the SLAM problem on Wikipedia [here](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping).  
* The SLAM algorithm used in this workshop is a Rao-Blackwellized Particle Filter ([this tutorial](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/rbpf-slam-tutorial-2007.pdf) has some more information on RBPFs).  
* The ROS implementation used here is called [GMapping](http://wiki.ros.org/gmapping), it was open sourced by Grisetti et al [here](https://openslam-org.github.io/gmapping.html), while [this paper](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf) describes the algorithm in detail.

### Navigation
* TODO

Instructions
----------------

### Overview

This session will build on the ROS packages Clearpath makes available for their [Husky robot base](http://wiki.ros.org/Robots/Husky).  You will install and configure the `gmapping` SLAM package, along with the `move_base` navigation package. 

### Setup

Make sure you are using the Catkin Workspace you created yesterday ([instructions here](https://github.com/ros-workshop/course)) and you have Git cloned this repository into the folder `workshop_ws/src`.

<details><summary>Click here to double check</summary>

Either: 
```
cd ~/workshop_ws/src
git clone https://github.com/ros-workshop/slam-navigation.git
```
Or if you are using SSH keys:
```
cd ~/workshop_ws/src
git clone git@github.com:ros-workshop/slam-navigation.git
```

</details>

You should have installed the Husky ROS packages `ros-kinetic-husky-simulator` and `ros-kinetic-husky-viz` yesterday. Install the `ros-kinetic-husky-navigation` package also.

<details><summary>Click here to double check</summary>

```
sudo apt install ros-kinetic-husky-simulator ros-kinetic-husky-viz ros-kinetic-husky-navigation
```

</details>
 


## Start the Gazebo simulation:

```
roslaunch husky_gazebo husky_playpen.launch # Launches Husky with SICK LiDAR in Gazebo
```

Note: This will take some time to start, as the simulator needs to download resources from the Gazebo servers.

Check the console for error messages.  

```
roslaunch slam_navigation husky_nav.launch # Launches Rviz
```


Hints:




Hint: if you're running in a virtual machine, slow the Rviz framerate down to 10 Hz (Expand `Global Options` in the Displays panel)
To load a different Gazebo world, e.g.: 
```
roslaunch husky_gazebo husky_empty_world.launch world_name:=/opt/ros/kinetic/share/jackal_gazebo/worlds/jackal_race.world
```

If Gazebo freezes, you may need to force kill it with `pkill gzserver`




<details><summary>Click here to cheat!</summary>

```
# Install wstool and rosdep.
  
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
  * **Hints:**
    * Clone and build Google's Cartographer by following the instructions [here](https://google-cartographer-ros.readthedocs.io/en/latest/)
    * **Warning:** Make sure you don't install Protobuf 3.x system wide (don't type sudo!) or you'll break other ROS packages. 
    * If you get stuck, Clearpath have done [some of the work for you.](https://github.com/husky/husky_cartographer_navigation/blob/master/husky_cartographer_install.sh)
    * Note: make sure you understand what `catkin_make_isolated` does (e.g. you'll need to run catkin_make more, )

### Questions
* What does the tf tree and node graphs look like while navigating?
* How do you remap a topic name when starting a node in a lanch file?
* What is an workspace overlay in Catkin? 

### Links
* [Clearpath Husky wiki](http://wiki.ros.org/Robots/Husky) 
* [Husky repos on Github](https://github.com/husky/husky)

