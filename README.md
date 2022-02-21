# SLAM & Navigation

## Overview

In this session we'll learn how to make a simulated [Clearpath Husky robot](http://wiki.ros.org/Robots/Husky) navigate smoothly between user-selected waypoints. 
We'll experiment with the [`gmapping`](http://wiki.ros.org/gmapping) SLAM package, along with the [`move_base`](http://wiki.ros.org/move_base) navigation package. 
At the end of this session you should have a simulated robot navigating around like this Youtube video (click to play):

<a href="http://www.youtube.com/watch?feature=player_embedded&v=WmGVRX2r8WY" target="_blank"><img src="http://img.youtube.com/vi/WmGVRX2r8WY/0.jpg" alt="Video" width="480" height="360" border="10" /></a>


## Background

### Simultaneous Localisation and Mapping (SLAM)
For a mobile robot to navigate an environment, it requires both 
1) A map of its environment, and 
2) Knowledge of where it is in that map. 
 
This is the _"Simultaneous Localisation and Mapping"_ or SLAM problem, which is a fundamental problem in robotics and the focus of considerable research over the last few decades. Spend a few minutes reading about:
* The [SLAM problem](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) on Wikipedia.  
* The [Rao-Blackwellized Particle Filter][rbpf-tutorial] (RBPF), which is the SLAM algorithm used in this workshop ([this tutorial][rbpf-tutorial] is a good starting point if you'd like to learn more).
* The [GMapping](http://wiki.ros.org/gmapping) ROS package, which is the RBPF implementation used in this workshop. [This paper](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf) by Grisetti et al. describes the approach in detail.
* Other solutions to the SLAM problem, e.g. [this slide deck][slam-tutorial] by Wolfram Burgard is a good general overview.

### Navigation

Once a robot has a map of it's environment and is well localised, it can begin planning paths to waypoint goals and navigating along them. Spend a few minutes reading about:
* [Robot navigation](https://en.wikipedia.org/wiki/Robot_navigation) and
* The [ROS Nav Stack](http://wiki.ros.org/navigation) including these [tutorials](http://wiki.ros.org/navigation/Tutorials).

## Workspace Setup

Make sure you are using the Catkin Workspace you created yesterday ([instructions here](https://github.com/ros-workshop/course)) and you have cloned this repository into the folder `workshop_ws/src`.

<details>
<summary>Click for a hint</summary>

Either: 
```bash
cd ~/workshop_ws/src
git clone https://github.com/ros-workshop/slam-navigation.git
```
Or if you are using SSH keys:
```bash
cd ~/workshop_ws/src
git clone git@github.com:ros-workshop/slam-navigation.git
```

</details>

The following Husky Debian packages should have been installed in yesterday's workshop session:
```bash
sudo apt install ros-$ROS_DISTRO-husky-simulator
sudo apt install ros-$ROS_DISTRO-husky-viz
``` 
For this session, install the `husky_navigation` package also:

```bash
sudo apt install ros-$ROS_DISTRO-husky-navigation 
```

**Note**: The `husky_navigation` package is dependent on the `gmapping` and `move_base` ROS packages that are used in this workshop and will be installed automatically. 


## Launching the SLAM and Navigation Stacks 

To jump start with a working configuration, this repository includes a customised version of Clearpath's `husky_navigation` package. 

First start four separate terminal windows (hint: consider using a split screen terminal such as `terminator`), and source your Catkin workspace

<details><summary>Click for a hint</summary>

```
cd ~/workshop_ws && source devel/setup.bash
```
</details>

Now, launch `gazebo`, `gmapping`, `rviz` and `move_base` in the four terminal windows:
1. Launch the `gazebo` Husky simulation environment:
    ```
    roslaunch husky_gazebo husky_playpen.launch
    ```
    * This command will start the roscore server and load the Gazebo world and robot model.
    * The Gazebo 3D assets should be already cached from yesterday's workshop session. If Gazebo appears to hang, the assets may be downloading in the background. Visit [this repo](https://github.com/osrf/gazebo_models) and download the assets (Hint: Use "Download ZIP"). To install, unzip the files into `~/.gazebo/models/`.
    * Consider arranging the Gazebo window so it fills the left half of the screen.
    * Check the console for error messages before proceeding.

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

In the `rviz` window, use the `2D Nav Goal` tool in the top toolbar to set a movement goal in the visualizer. 
Click and drag to set the desired heading.
As the robot navigates to the goal, you should see the occupancy gridmap grow. 
* Try navigating a few big loops around the simulated environment: 
  * The system will perform badly and you should see errors in the gridmap that continue to grow; we'll look into this in the next section.
* Try instructing the Husky to drive close around an obstacle:
  * You might notice it approaching too close (or crashing and/or flipping!); we'll look at this later also

### Hints:
* To help with performance in a virtual machine, you might want to slow the Rviz framerate down to 10 Hz (Expand `Global Options` in the Displays panel).
Conversely, increase this to ~30 Hz on a fast PC. 
* If Gazebo freezes, you may need to force kill it with `pkill gzserver`.
* During development, you can stop and restart `gmapping` and `move_base` independently with CTRL+C.
* The lidar's range is deliberately foreshortened for this session (lidar-based SLAM is easy when you can see four walls)
* If `rviz` appears cluttered, feel free to turn off the Sensing group of visualisers in the Displays panel

## Exploring SLAM using `gmapping` 

All real-world sensor data is noisy, thus, when a robot drives around fusing its noisy sensor data into a map and pose estimate (the SLAM problem), the map and pose will be noisy and accumulate drift.
Unfortunately, the types of errors experienced by wheel odometry and lidar scan matching are frequently *non-Gaussian*, while process and measurement models are typically nonlinear, which means multivariate Gaussian probablility distributions (e.g. the frequently used Extended Kalman Filter, or EKF) are badly suited to representing a robots pose and map uncertainty.

`gmapping` uses a Rao-Blackwellized Particle Filter to represent the current hypotheses of the robot's trajectory.
Here, dozens of particles (or more) work together to describe complex probability distributions that are non-Gaussian and can handle nonlinear process and measurement models.
However, for a fixed size set of particles, there is a maximum number of hypotheses that can be represented.
Use Google to spend a few minutes learning about "[sample starvation](http://lmgtfy.com/?q=sample+starvation+particle+filter+slam)".

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

### Task 2: Computational requirements
What is `gmapping's` CPU usage before and after the change? 
* Hint: use `htop` and make sure Gazebo is running at 1.0x realtime in both instances. 
* With particle filters, there's a direct (linear) trade between the number of particles and CPU usage.
This will directly affect the size of the environment that `gmapping` can handle.
`gmapping` struggles to perform loop closures and maintain map accuracy when exploring dozens of meters 'open loop' (not revising previously seen parts of the map). 

### Task 3: Loop closures and transforms
* With a clean gridmap, navigate the simulated Husky around the outside of the environment again.
* Try to observe what happens when a previously visited part of the map is revisited after a long excursion. 
* If you notice a jump in the robot's pose, this is a classical "loop closure".
The `gmapping` algorithm will ocassionally perform small loop closures like this.
* Change the Fixed Frame in `Global Options` in the Displays panel to "odom" instead of "map"
   * Perform another loop closure and observe what happens. Why does the gridmap jump around instead?
   * ROS specifies that the odom->base_link transform is continuous. Take a look at [REP 105](http://www.ros.org/reps/rep-0105.html) for more information.  
   * Loop closures create a discrete jump in the map->odom transform.
   * Tune the particle count to balance CPU usage vs. ability to perform loop closures.
 
## Exploring Navigation with `move_base` 

While there are dozens of navigation algorithms described in the literature (`move_base` only implements a few), there exists a handful of commonly occuring parameters.
We'll explore two of them here. 

**Note:** if at any time the gridmap gets badly distorted, restart  `gmapping` by hitting CTRL+C in its terminal window.
`move_base` will not be able to create global plans if the gridmap is distorted. 

### Task 1: Obstacle avoidance

Tell the simulated Husky to navigate near to and/or around an obstacle:  
  * You might notice that it approaches closer than you'd like, and occasionally hits obstacles below its lidar's height
  * Read about obstacle "inflation" [here](http://wiki.ros.org/costmap_2d). 
    * Take note of the plot showing how cell cost decreases with the distance from an obstacle.
    * Note the difference between the "lethal" distance where the robot would be in collision, and the "inscribed" and "possibly circumscribed" distances that depend on the robot's footprint. 
    * Look for a parameter that might create a "buffer zone" around obstacles to naturally keep the robot further away. 
    * Find and tune this parameter in the `move_base` configuration files. 
 
<details><summary>Click for a hint</summary>
    
   * The parameter affects both local and global cost maps, look in the `move_base.launch` file
   
<details><summary>Click to cheat</summary>

    * The particular line to consider is in the `config/costmap_common.yaml` file [here](https://github.com/ros-workshop/slam-navigation/blob/master/slam_navigation/config/costmap_common.yaml#L24)

</details>

</details>

  * After adjusting this parameter, both the local and global cost maps should show a larger buffer around obstacles in `rviz` 

### Task 2: Goal tolerances 
Tell the simulated Husky to navigate to a waypoint:  
* You might notice that the simulated Husky doesn't stop completely after reaching its goal. 
* This "hunting" behaviour is common, and relates to the goal tolerance in the local planner.
* Find the parameter file that configures the local planner and adjust it appropriately. 

<details>
    <summary>Click for a hint</summary>
    
   * In the `move_base.launch` file, look for the line `<arg name="base_local_planner" default="dwa_local_planner/DWAPlannerROS"/>`
    * This maps to the local planner being used and hints at the config file section

<details>
    <summary>Click to cheat</summary>

    * The particular lines to consider are in `config/planner.yaml` file [here](https://github.com/ros-workshop/slam-navigation/blob/master/slam_navigation/config/planner.yaml#L72)

</details>

</details>

* There is another trade off here between how close you want your robot to achieve its target, vs. how many three-point turns it performs trying to navigate accurately!


## Try this next
* **Try increase the maximum lidar range in `gmapping`**
  * The default in `husky_gmapping.launch` is six meters
  * What happens when the lidar can hit all four walls at the same time? 
  * Does the CPU usage increase? Can the particle count be decreased?  
* **Try a different Gazebo world:**  
  * Feel free to find alternative `.world` files on the internet. 
  * Launch Gazebo with:
     ```
     roslaunch husky_gazebo husky_empty_world.launch your_custom.world
     ```
  * E.g. you can install the `jackal_race.world` file with `sudo apt install ros-noetic-jackal-gazebo` and launch Gazebo with:
     ```
     roslaunch husky_gazebo husky_empty_world.launch \
             world_name:=/opt/ros/noetic/share/jackal_gazebo/worlds/jackal_race.world
     ```

## Stretch Goals
### Try geofencing your robot 
* **Motivation:** We want to annotate the map to keep the robot in a particular area 
* **Goal:** Save the map to disk, edit it, and then relocalising and navigate in it
* **Instructions:**
  * Build a complete map of the environment using `gmapping`
  * Save the gridmap to disk (hint: google `ros map_server map_saver`) and shut down `gmapping`
  * Edit the gridmap (e.g. GIMP) to add some virtual "fences" 
  * Load the map and use the `amcl` package to relocalise the robot (hint: start [here](http://wiki.ros.org/husky_navigation/Tutorials/Husky%20AMCL%20Demo))
  * Show that `move_base` can navigate without crossing your virtual fences 
### Add a Velodyne VLP-16 lidar
* **Motivation:** A 16-plane lidar scanner provides a comprehensive 3D view of the world, however the extra data doesn't work out of the box with `gmapping` and `move_base`.  
* **Goal:** Integrate a VLP-16 into Gazebo and configure it to work with `gmapping` and `move_base`.
* **Hints:**
  * Integrate into Gazebo (including the URDF)
  * The VLP-16's point cloud will need to be squashed into a laser scan topic for `gmapping`
  * Lidar returns from the ground can appear like walls. What is the best way to filter them?
  * The same technique can be used for `move_base`, however you could explore [this package.](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)
### Large-scale SLAM with Cartographer 
* **Motivation:** The RBPF algorithm used in `gmapping` does not scale well
* **Goal:** Explore [Cartographer][cartographer], a modern pose-graph based SLAM implementation 
* **Instructions and Hints:**
  * Melodic users should be able to `apt install ros-melodic-cartographer-ros`
  * Noetic users will need to clone and build Cartographer by following the instructions [here][cartographer], note: 
    * Make sure you understand what `catkin_make_isolated` does if you're actively developing in a workspace (you've been warned!)
    * You can use `catkin build` instead of `catkin_make_isolated`, and you can put it into your active workspace, however it is better to learn how to *extend* a Catkin workspace!
    * If you get stuck, Clearpath have done [some of the work for you.](https://github.com/husky/husky_cartographer_navigation/blob/master/husky_cartographer_install.sh)
  * Pose-graph SLAM can require a lot of computation, so computation may lag behind realtime.
### Try navigating a real robot 
* **Motivation:** simulated robots often miss some of the subtleties of real robots   
* **Goal:** configure a [TurtleBot](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) to navigate around the lab
* **Instructions:**
  * There are limited TurtleBots available; please demonstrate navigation in the Gazebo simulation before borrowing a TurtleBot. 
  * Follow the [instructions here](http://emanual.robotis.com/docs/en/platform/turtlebot3/navigation)

### Questions
* What does the `tf` tree and node graphs look like while navigating?
* How do you remap a topic name when starting a node in a lanch file?
* What is an workspace overlay in Catkin? 
* What is the difference between a ROS package and a Debian (Ubuntu) package?

### Links
* [Clearpath Husky wiki](http://wiki.ros.org/Robots/Husky)
* [Husky repositories on Github](https://github.com/husky/husky)

[rbpf-tutorial]: http://www2.informatik.uni-freiburg.de/~stachnis/pdf/rbpf-slam-tutorial-2007.pdf
[slam-tutorial]: https://www.rsj.or.jp/databox/international/iros16tutorial_1.pdf
[cartographer]: https://google-cartographer-ros.readthedocs.io/en/latest