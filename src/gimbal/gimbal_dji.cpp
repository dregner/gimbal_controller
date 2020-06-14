//
// Created by vant3d on 14/06/2020.
//
// System includes
#include "unistd.h"
#include <cstdint>
#include <iostream>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/Gimbal.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>


// DJI OSDK includes
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_gimbal.hpp>

// DARKNET includes
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

// Be precise here

class ControlGimbal_dji {
private:

    //ROS Node
    ros::NodeHandle nh;

    //Publisher and Subscribers
    geometry_msgs::Vector3Stamped gimbal_angle;
    DJI::OSDK::Gimbal::SpeedData gimbalSpeed;
    DJI::OSDK::Vehicle vehicle;

    ros::Subscriber gimbal_angle_subscriber;
    ros::Publisher gimbal_angle_cmd_publisher;
    ros::Publisher gimbal_speed_cmd_publisher;
    ros::Subscriber sub_boundingboxes;
    ros::Subscriber sub_found_object;

    float roll;
    float pitch;
    float yaw;

public:
    ControlGimbal_dji() {

        // ROS stuff
        gimbal_angle_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>
                ("dji_sdk/gimbal_angle", 10, &ControlGimbal_dji::gimbalAngleCallback, this);
        sub_boundingboxes = nh.subscribe<darknet_ros_msgs::BoundingBoxes>
                ("darknet_ros/bounding_box", 10, &ControlGimbal_dji::ReadBb, this);
        sub_found_object = nh.subscribe
                ("darknet_ros/found_object", 10, &ControlGimbal_dji::Found_obj, this);
        gimbal_angle_cmd_publisher = nh.advertise<dji_sdk::Gimbal>
                ("dji_sdk/gimbal_angle_cmd", 10);
        gimbal_speed_cmd_publisher = nh.advertise<geometry_msgs::Vector3Stamped>
                ("dji_sdk/gimbal_speed_cmd", 10);

    }

    ~ControlGimbal_dji() {}

    void ReadBb(){

    }
    void Found_obj(){

    }
    void setGimbalSeed() {
        gimbalSpeed.roll = 100;
        gimbalSpeed.pitch = 50;
        gimbalSpeed.yaw = -200;
        gimbalSpeed.gimbal_control_authority = 1;
        gimbalSpeed.disable_fov_zoom = 0;
        gimbalSpeed.ignore_user_stick = 0;
        gimbalSpeed.extend_control_range = 0;
        gimbalSpeed.ignore_aircraft_motion = 0;
        gimbalSpeed.yaw_return_neutral = 0;
        gimbalSpeed.reserved0 = 0;
        gimbalSpeed.reserved1 = 0;

        vehicle.gimbal->setSpeed(&gimbalSpeed);
    }

    void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        gimbal_angle = *msg;
        roll = gimbal_angle.vector.y;
        pitch = gimbal_angle.vector.x;
        yaw = gimbal_angle.vector.z;
    }
};
//struct RotationAngle{
//    float roll;
//    float pitch;
//    float yaw;
//};
//
//struct GimbalContainer{
//    int roll = 0;
//    int pitch = 0;
//    int yaw = 0;
//    int duration = 0;
//    int isAbsolute = 0;
//    bool yaw_cmd_ignore = false;
//    bool pitch_cmd_ignore = false;
//    bool roll_cmd_ignore = false;
//    RotationAngle initialAngle;
//    RotationAngle currentAngle;
//    GimbalContainer( int roll = 0,
//                     int pitch = 0,
//                     int yaw = 0,
//                     int duration = 0,
//                     int isAbsolute = 0,
//                     RotationAngle initialAngle = {},
//                     RotationAngle currentAngle = {}):
//            roll(roll), pitch(pitch), yaw(yaw),
//            duration(duration),isAbsolute(isAbsolute),
//            initialAngle(initialAngle), currentAngle(currentAngle){}
//};

int main(int argc, char **argv) {

    ros::init(argc, argv, "gimbal_track");
    ControlGimbal_dji control;
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}