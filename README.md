# SLAM & Navigation

**Goal:** At the end of this session you should have a simulated robot navigating smoothly to user-selected waypoints, like this randomly selected Youtube video:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=WmGVRX2r8WY" target="_blank"><img src="http://img.youtube.com/vi/WmGVRX2r8WY/0.jpg" alt="Video" width="480" height="360" border="10" /></a>

**Overview:** This session uses the ROS packages Clearpath makes available for their [Husky robot base](http://wiki.ros.org/Robots/Husky).  You will experiment with the `gmapping` SLAM package, along with the `move_base` navigation package.

## Background

### Simultaneous Localisation and Mapping (SLAM)
For a mobile robot to navigate an environment, it requires both 1) a map of its environment, and 2) knowledge of where it is in that map. This is the "Simultaneous Localisation and Mapping" or SLAM problem, which is a fundamental problem in robotics and the focus of considerable research over the last few decades.
* Spend a few minutes reading about the SLAM problem on Wikipedia [here](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping).  
* The SLAM algorithm used in this workshop is a Rao-Blackwellized Particle Filter ([this tutorial](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/rbpf-slam-tutorial-2007.pdf) has some more information on RBPFs).  
* The ROS implementation used here is called [GMapping](http://wiki.ros.org/gmapping), it was open sourced by Grisetti et al. [here](https://openslam-org.github.io/gmapping.html), while [this paper](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf) describes the algorithm in detail.

### Navigation
* TODO

## Workspace Setup

* Make sure you are using the Catkin Workspace you created yesterday ([instructions here](https://github.com/ros-workshop/course)) and you have Git cloned this repository into the folder `workshop_ws/src`.

<details>
<summary>Click for a hint</summary>

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


* You should have installed the Husky Debian packages `ros-kinetic-husky-simulator` and `ros-kinetic-husky-viz` yesterday using `apt`. For this session, install the `ros-kinetic-husky-navigation` package also.

<details><summary>Click for a hint</summary>

```
sudo apt install ros-kinetic-husky-simulator ros-kinetic-husky-viz ros-kinetic-husky-navigation
```

</details>


* The `ros-kinetic-husky-navigation` Debian package is dependent on the `gmapping` and `move_base` ROS packages that are used in this workshop, they will be installed automatically. 


## Launching the SLAM and Navigation Stacks 

To jump start a working configuration, this repository includes a customised version of Clearpath's `husky_navigation` package. 

First start four separate terminal windows (hint: consider using a split screen terminal such as `apt install terminator`), and source your Catkin workspace

<details><summary>Click for a hint</summary>

```
cd ~/workshop_ws && source devel/setup.bash
```
</details>

Now, launch `gazebo`, `gmapping` `rviz` and `move_base` in the four terminal windows:
1. Launch the Husky simulation environment:
    ```
    roslaunch husky_gazebo husky_playpen.launch
    ```
    * Note: 
      * This will take several minutes to start on first run, as the simulator needs to download resources from the internet
      * This command will start the roscore server
      * Consider arranging this window so Gazebo fills the left half of the screen
      * Check the console for error messages before proceeding
1. Launch `gmapping`:
    ```
    roslaunch slam_navigation husky_gmapping.launch
    ```
    * Note: The `gmapping` occupancy gridmap output is shown in `rviz`:
      * Black cells are obstacles
      * Light grey cells are free space
      * Dark grey cells are unknown
      * Grey cells are unknown
1. Launch `rviz` for visualisation:
    ```
    roslaunch slam_navigation husky_rviz.launch
    ```
    * Note: 
        * Consider arranging this window so it fills the right half of the screen
        * Make sure Sensing group of visualisers in are enabled in the Displays panel.
1. Launch `move_base`:
    ```
    roslaunch slam_navigation husky_move_base.launch
    ```
    * Note: In `rviz`, make sure Navigation group of visualisers in are enabled in the Displays panel.

### Basic Navigation: 

In the `rviz` window, use the `2D Nav Goal` tool in the top toolbar to set a movement goal in the visualizer. Click and drag to set the desired heading.  As the robot navigates to the goal, you should see the occupancy gridmap grow. 
* Try navigating a few big loops around the simulated environment: 
  * The system will perform badly and you should see errors in the gridmap that continue to grow; we'll look into this in the next section.
* Try instructing the Husky to drive close around an obstacle:
  * You might notice it approaching too close (or crashing and/or flipping!); we'll look at this later also

### Hints:
* To help with performance in a virtual machine, you might want slow slow the Rviz framerate down to 10 Hz (Expand `Global Options` in the Displays panel). Conversely, increase this to ~30 Hz on a fast PC. 
* If Gazebo freezes, you may need to force kill it with `pkill gzserver`.
* During development, you can stop and restart `gmapping` and `move_base` independently with CTRL+C.
* The lidar's range is deliberately foreshortened for this session (lidar-based SLAM is easy when you can see four walls)
* If `rviz` appears cluttered, feel free to turn off the Sensing group of visualisers in the Displays panel

## Exploring SLAM using `gmapping` 

All real-world sensor data is noisy, thus, when a robot drives around fusing its noisy sensor data into a map and pose estimate (the SLAM problem), the map and pose will be noisy and accumulate drift. Unfortunately, the types of errors experienced by wheel odometry and lidar scan matching are frequently *non-Gaussian*, while process and measurement models are typically nonlinear, which means multivariate Gaussian probablility distributions (e.g. the often used Extended Kalman Filter, or EKF) are badly suited to representing a robots pose and map uncertainty.

`gmapping` uses a Rao-Blackwellized Particle Filter to represent the current hypotheses of the robot's trajectory. Here, dozens of particles (or more) work together to describe complex probability distributions that are non-Gaussian and can handle nonlinear process and measurement models. However, for a fixed size set of particles, there is a maximum number of hypotheses that can be represented. Use Google to spend a few minutes learning about "[sample starvation](http://lmgtfy.com/?q=sample+starvation+particle+filter+slam)".

**Note:** if at any time the gridmap gets badly distorted, restart  `gmapping` by hitting CTRL+C in its terminal window. `move_base` will not be able to create global plans if the gridmap is distorted. 

### Task 1: Drift, or accumulated sensor errors 
Does the current `gmapping` configuration look like it is handling drift? 
* After driving around the simulated environment, errors appear to accumulate without bound.
* Check if the `gmapping` configuration could be experiencing sample starvation.
* Change one of the parameters and restart `gmapping` to see if the drift is corrected and map errors reduced.

<details><summary>Click for a hint</summary>
   Look at the parameters in the `husky_gmapping.launch` file
<details><summary>Click to cheat</summary>
  The particular line to consider in the `husky_gmapping.launch` file is [here](https://github.com/ros-workshop/slam-navigation/blob/master/slam_navigation/launch/husky_gmapping.launch#L40)
</details>
</details>

### Task 2: CPU usage
What is `gmapping's` CPU usage before and after the change? 
* Hint: use `htop` and make sure Gazebo is a 1.0x realtime for both. 
* With particle filters, there's a direct (linear) trade between the number of particles and CPU usage. This will directly affect the size of the environment that `gmapping` can handle. `gmapping` struggles to perform loop closures and maintain map accuracy when exploring dozens of meters 'open loop' (not revising previously seen parts of the map). 

### Task 3: Loop closures and transforms
* With a clean gridmap, navigate the simulated Husky around the outside of the environment again.
* Try to observe what happens when a previously visited part of the map is revisited after a long excursion. 
* If you notice a jump in the robot's pose, this is a classical "loop closure". The `gmapping` algorithm will ocassionally perform small loop closures like this.
* Change the Fixed Frame in `Global Options` in the Displays panel to "odom" instead of "map"
   * Perform another loop closure and observe what happens. 
   * ROS specifies that the odom->base_link transform is continuous. Take a look at [REP 105](http://www.ros.org/reps/rep-0105.html) for more information.  
   * Loop closures create a discrete jump in the map->odom transform.
   * Tune the particle count to balance CPU usage vs. ability to perform loop closures.
 
## Exploring Navigation with `move_base` 

While there are dozens of navigation algorithms described in the literature (`move_base` only implements a few), there exists a handful of commonly occuring parameters. We'll explore two of them here. 

### Task 1: Goal tolerances 
Give the Husky a waypoint to navigate to  
* You might notice that the simulated Husky doesn't stop completely after reaching its goal. This "hunting" behaviour is common, and relates to the goal tolerance in the local planner.
* Find the parameter file that configures the local planner and adjust it appropriately. 

<details><summary>Click for a hint</summary>
   In the `move_base.launch` file, look for the line
    ```xml
      <arg name="base_local_planner" default="dwa_local_planner/DWAPlannerROS"/>
    ```
    This maps to the local planner being used and hints at the config file section
<details><summary>Click to cheat</summary>
  The particular lines to consider are in the `config/planner.yaml` file is [here](https://github.com/ros-workshop/slam-navigation/blob/master/slam_navigation/config/planner.yaml#L72)
</details>
</details>

* There is another trade off here between how close you want your robot to achieve its target, vs. how many three-point turns it performs trying to navigate accurately. 

### Task 2: Obstacle avoidance
* Try instructing the Husky to drive close around an obstacle:
  * You might notice it approaching too close (or crashing and/or flipping!); we'll look at this later also


too close to obstacles

http://wiki.ros.org/costmap_2d



### Stretch Goals
* velodyne TODO
* increase lidar range
* Later, try loading a different Gazebo world with, e.g.: 
     ```
     roslaunch husky_gazebo husky_empty_world.launch \
             world_name:=/opt/ros/kinetic/share/jackal_gazebo/worlds/jackal_race.world
     ```
  * Note: the `jackal_race.world` file is found in `sudo apt install ros-kinetic-jackal-gazebo`

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
    * Build a complete map of the environment using `gmapping`
    * Save the gridmap to disk (hint: google `ros map_server map_saver`) and shut down `gmapping`
    * Edit the gridmap (e.g. GIMP) to add some virtual "fences" 
    * Load the map and use the `amcl` package to relocalise the robot (hint: start [here](http://wiki.ros.org/husky_navigation/Tutorials/Husky%20AMCL%20Demo))
    * Show that `move_base` can navigate without crossing your virtual fences 
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
* What is the difference between a ROS package and a Debian (Ubuntu) package?

### Links
* [Clearpath Husky wiki](http://wiki.ros.org/Robots/Husky) 
* [Husky repos on Github](https://github.com/husky/husky)

