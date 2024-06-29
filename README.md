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
    git clone <repository-url>
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

    ## Doing AruCo Detection in Gazebo

   1. Add camera to 
