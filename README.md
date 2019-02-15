SLAM & Navigation
==================
Material for day 2 of the ROS Workshop

Introduction
-------------

### Simultaneous Localisation and Mapping (SLAM)
For a mobile robot to navigate through its environment, it requires both 1) a map and 2) knowledge of where it is in that map. This is the "Simultaneous Localisation and Mapping" or SLAM problem, which has been a core problem in robotics over the last few decades.
* Spend a few minutes reading about the SLAM problem on Wikipedia [here](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping).  
* The SLAM algorithm used in this workshop is a Rao-Blackwellized Particle Filter ([this tutorial](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/rbpf-slam-tutorial-2007.pdf) has some more information on RBPFs).  
* The ROS implementation used here is called [GMapping](http://wiki.ros.org/gmapping), it was open sourced by Grisetti et al [here](https://openslam-org.github.io/gmapping.html), while [this paper](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf) describes the algorithm in detail. 

### Navigation
* 

Instructions
----------------

### GMapping for SLAM

http://wiki.ros.org/gmapping



### Move_base for Navigation

<details><summary>d</summary>
test
</details>

### Stretch Goals
* **Geofencing your robot:** this requires saving the map to disk, editing it, and then relocalising in it
  * Build a complete map of the environment using ```gmapping```
  * Save the gridmap to disk and shut down ```gmapping```
  * Edit the gridmap (e.g. GIMP) to add some virtual "fences" 
  * Load the map and use the ```amcl``` package to relocalise the robot
  * Show that ```move_base``` can navigate without crossing your virtual fences 

### Questions
* What does the tf tree and node graphs look like while navigating?
