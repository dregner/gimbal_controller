//
// Created by vant3d on 05/07/2020.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <fstream>

#define RAD2DEG(RAD) ((RAD) * (180.0) / (M_PI))


uint8_t mode;
float width, height;
using namespace std;



void callback(const sensor_msgs::Image::ConstPtr &msg) {
    height = msg->height;
    width = msg->width;
    cout << "Width: " << width << "\tHeight: " << height << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "read_main_cam_image");
    ros::NodeHandle nh;

    // ROS stuff
    ros::Subscriber gimbal_angle_subscriber = nh.subscribe("dji_sdk/main_camera_images", 10, &callback);
    while (ros::ok()) {
        ros::spin();
    }
}


