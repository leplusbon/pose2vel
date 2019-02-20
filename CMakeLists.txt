cmake_minimum_required(VERSION 2.8.3)
project(pose2vel)
find_package(catkin REQUIRED cmake_modules)
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp rospy std_msgs std_srvs genmsg
	tf2 tf2_ros
)
find_package(Eigen REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include include)
include_directories(${Eigen_INCLUDE_DIRS})

add_executable(pose2vel src/main.cpp)
target_link_libraries(pose2vel ${catkin_LIBRARIES})
