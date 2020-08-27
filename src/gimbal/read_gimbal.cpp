//
// Created by vant3d on 05/07/2020.
//

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <dji_sdk/Gimbal.h>
#include <bitset>
#include <iostream>
#include <fstream>

#define RAD2DEG(RAD) ((RAD) * (180.0) / (M_PI))


uint8_t mode;
float roll, pitch, yaw;
using namespace std;
static std::ofstream gimbal_angles;
int init_time;
void print() {

    cout << "Time: " << init_time - ros::Time::now().sec << " seconds" << endl;
    cout << "Roll: " << roll << "\tPitch: " << pitch << "\tYaw: " << yaw << endl;
    cout << "\033[2J\033[1;1H";     // clear terminal

}

void gimbalAngleCallBack(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
    roll = msg->vector.y;
    pitch = msg->vector.x;
    yaw = 111.6 - msg->vector.z;

    print();
    if(gimbal_angles.is_open()){
        int dt = init_time - ros::Time::now().sec;
        gimbal_angles << dt << "\t" << msg->vector.y << "\t" << msg->vector.x << "\t" << msg->vector.z <<endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "read_sdk_gimbal");
    ros::NodeHandle nh;

    // ROS stuff
    init_time = ros::Time::now().sec;
    ros::Subscriber gimbal_angle_subscriber = nh.subscribe("dji_sdk/gimbal_angle", 10, &gimbalAngleCallBack);
    gimbal_angles.open("sdk_gimbal_angle.txt");
    while (ros::ok()) {
        ros::spin();
    }
}


