cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gimbal_controller)

#Adicionando os plugins ROS

find_package(roscpp REQUIRED)
find_package(rospy REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(Python COMPONENTS Interpreter Development)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

## Adicionado a biblioteca do OpenCV
find_package(OpenCV REQUIRED)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        cv_bridge
        genmsg
        image_transport
        std_msgs
        geometry_msgs
        message_generation
        dji_sdk
        darknet_ros_msgs)

find_package(DJIOSDK REQUIRED)

find_package(darknet_ros QUIET)
if(darknet_ros_FOUND)
    message(STATUS "Found darknet_ros package, will use it for object depth perception demo.")
    add_definitions(-DUSE_DARKNET_ROS)
endif()

find_package(OpenCV 3.3.0 QUIET)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    add_definitions(-DOPEN_CV_INSTALLED)
    set(OPEN_CV_3_3_0_INSTALLED TRUE)

    find_package(OpenCVContribImgProc QUIET)

    find_package(OpenCVViz QUIET)
else()
    find_package(OpenCV QUIET )
    if (OpenCV_FOUND)
        message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, depth percpetion demo requires 3.3.0+ for better experience")
        message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
        add_definitions(-DOPEN_CV_INSTALLED)
    else()
        message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
    endif()
endif()


find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_GPU)
elseif()
    message(STATUS "Did not find CUDA in the system")
endif()


add_message_files(
        FILES
        BoundingBox.msg
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        geometry_msgs
        sensor_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        rospy
        actionlib_msgs
        geometry_msgs
        sensor_msgs
        message_runtime
        std_msgs
        darknet_ros
        darknet_ros_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})



foreach(dir launch models urdf worlds config)
    install(DIRECTORY ${dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)



##########  LIBRARIES #############

# Library de mover a parede
add_library(animated_ball SHARED src/animate/animated_ball.cc src/darknet/darknet_bb.cpp)
target_link_libraries(animated_ball ${GAZEBO_LIBRARIES})


##########  EXECUTABLES ##########

#add_executable(g_control src/gimbal/control_joint.cpp)
#target_link_libraries(g_control ${catkin_LIBRARIES})


add_executable(pub_camera src/opencv/pub_camera.cpp)
target_link_libraries(pub_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(inv_kinematic src/gimbal/inverse_kinematic.cpp src/GetParams.cpp)
#add_dependencies(inv_kinematic ${gimbal_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inv_kinematic ${catkin_LIBRARIES})

add_executable(gimbal_dji src/gimbal/gimbal_dji.cpp)
add_dependencies(gimbal_dji dji_sdk_generate_messages_cpp)
target_link_libraries(gimbal_dji
                        ${catkin_LIBRARIES}
                        ${DJIOSDK_LIBRARIES})

add_executable(read_bb src/darknet/darknet_bb.cpp)
target_link_libraries(read_bb ${catkin_LIBRARIES})