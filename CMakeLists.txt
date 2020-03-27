cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gimbal_controller)

#Adicionando os plugins ROS

find_package(roscpp REQUIRED)
find_package(rospy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Python COMPONENTS Interpreter Development)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

## Adicionado a biblioteca do OpenCV
find_package(OpenCV REQUIRED)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp rospy sensor_msgs cv_bridge
        genmsg
        image_transport
        pcl_conversions
        pcl_msgs
        pcl_ros
        std_msgs)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        gazebo_msgs
        roscpp
        rospy
        sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})


foreach(dir launch models urdf worlds config)
    install(DIRECTORY ${dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

        #############
    ## LIBRARIES
##########

# Library de mover a parede
add_library(animated_ball SHARED src/animated_ball.cc)
target_link_libraries(animated_ball ${GAZEBO_LIBRARIES})


            ###########
    ## EXECUTABLES
###########

add_executable(g_control src/control_joint.cpp)
target_link_libraries(g_control ${catkin_LIBRARIES})



