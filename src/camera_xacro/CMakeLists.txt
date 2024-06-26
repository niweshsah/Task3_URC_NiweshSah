# cmake_minimum_required(VERSION 3.0.2)
# project(camera_xacro)

# ## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   sensor_msgs
#   cv_bridge
#   image_transport
# )

# ## Declare a catkin package
# catkin_package(
#   CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport
# )

# ## Add Python scripts
# catkin_install_python(PROGRAMS
#   scripts/aruco_detector.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


cmake_minimum_required(VERSION 3.0.2)
project(camera_xacro)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_detector_cpp src/aruco_detector_cpp.C)

target_link_libraries(aruco_detector_cpp
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
