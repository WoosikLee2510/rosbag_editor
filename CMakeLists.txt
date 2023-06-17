cmake_minimum_required(VERSION 3.5.1)
project(rosbag_editor)

# To transform the frames
find_package(Eigen3 REQUIRED)

# Find ROS build system
find_package(catkin QUIET COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs image_geometry visualization_msgs image_transport cv_bridge pcl_ros)

catkin_package()

# Include our header files
include_directories( include ${catkin_INCLUDE_DIRS})

##################################################
# Make binary files!
##################################################
add_executable(rosbag_editor rosbag_editor.cpp)
target_link_libraries(rosbag_editor ${catkin_LIBRARIES})
install(TARGETS rosbag_editor
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )