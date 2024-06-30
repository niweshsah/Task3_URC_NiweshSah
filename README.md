## Prerequisites

Before you can run the node, make sure you have the following installed:
- [ROS (Robot Operating System)](http://wiki.ros.org/ROS/Installation)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)



**NOTE:** We are using ROS Noetic

## Installation

1.  **Make workspace (e.g. catkin_ws)**

     ```bash
    cd ~
    mkdir catkin_ws
    cd catkin_ws/
    ```

2.  **Clone the Repository**

    ```bash
    git clone https://github.com/niweshsah/Task3_URC_NiweshSah.git
    ```

3. **Build the Package**

    Ensure you are in the workspace root directory (e.g., `catkin_ws`).

    ```bash
    catkin init
    catkin build
    ```
4. **Sourcing and setting the environment variables**:

     Type " nano ~/.bashrc " in terminal and add following lines at the end:

     ```bash
    source /opt/ros/noetic/setup.bash
     export TURTLEBOT3_MODEL=burger
     source ~/catkin_ws/devel/setup.bash
     ```

     Save the file and open the terminal:
     ```bash
    source ~/.bashrc
    ```

5. **Adding camera to turtlebot3 burger**

   Open terminal:
   
     ```bash
     roscd turtlebot3_description/
     cd urdf/
     ```

    Open " turtlebot3_burger.urdf.xacro " file and add the following line and sudo save it:

     ```bash
     <xacro:include filename="$(find camera_xacro)/urdf/camera.xacro"/>
     ```

6. **Move aruco model file to new location**

   Open terminal:

   ```bash
     mv ~/catkin_ws/src/marker_0/  ~/.gazebo/models/
   ```
   

 ## Doing Aruco Detection in Gazebo

1. **Open Gazebo world with Turtlebot**
  
   Open terminal and type:

     ```bash
     cd ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/
     roslaunch turtlebot3_empty_world.launch 
     ```

2. **Insert aruco marker**

   In the gazebo world, go to " Insert " tab and click the "marker_0" model. Adjust the position of "Marker0" model in the Gazebo world.


3. **Adjust the robot using Teleoperation node**

   Open terminal and type:

   ```bash
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch  
   ```

   Adjust the pose of robot and you should be able to detect the inserted aruco marker.


## Adding custom Aruco Detection code 

1. Make your python/C++ code for aruco code detection (We wrote a C++ code). Make it executable and save it
2. Rebuild and source the workspace.
3. Add the following line in the launch file

```bash
  <node name="aruco_detector_cpp" pkg="camera_xacro" type="aruco_detector_cpp" output="screen" />  
```
4. Now follow the above given steps to run aruco detection in Gazebo Simulation.


## Demo Video

Refer to this [video](https://youtu.be/gz6GUQ0oo8Q).

**Recommended:** Watch this at 2x.

## Resources Used

1. https://classic.gazebosim.org/tutorials?tut=ros_gzplugins
2. https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
3. https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-1-display-real-time-video-feed-a98c078c708b
