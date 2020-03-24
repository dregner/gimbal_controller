cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gimbal_controller)

#Adicionando os plugins ROS
find_package(roscpp REQUIRED)
find_package(rospy REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
#
#install(DIRECTORY config
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
#install(DIRECTORY launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
#install(DIRECTORY urdf
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
#install(DIRECTORY worlds
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp rospy sensor_msgs cv_bridge
        genmsg
        image_transport
        pcl_conversions
        pcl_msgs
        pcl_ros
        std_msgs)

catkin_package(
        CATKIN_DEPENDS
        gazebo_msgs
        roscpp
        sensor_msgs
)

# Librery de mover a parede
add_library(animated_ball SHARED src/animated_ball.cc)
target_link_libraries(animated_ball ${GAZEBO_LIBRARIES})


foreach(dir launch models meshes urdf worlds config)
    install(DIRECTORY ${dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
