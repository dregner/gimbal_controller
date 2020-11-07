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
#define DEG2RAD(DEG) ((DEG) * (M_PI) / (180.0))


uint8_t mode;
float roll, pitch, yaw;
using namespace std;
static std::ofstream gimbal_pose;
double start, time_step;
void print() {

    cout << "Time: " << time_step << " seconds" << endl;
    cout << "Roll: " << roll << "\tPitch: " << pitch << "\tYaw: " << yaw << " deg" << endl;
    cout << "Roll: " << DEG2RAD(roll) << "\tPitch: " << DEG2RAD(pitch) << "\tYaw: " << DEG2RAD(yaw) << " rad" << endl;
    cout << "\033[2J\033[1;1H";     // clear terminal

}
void save_txt() {
    if (gimbal_pose.is_open()) {
        time_step = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec - start;
        gimbal_pose << time_step << "\t" << DEG2RAD(roll) << "\t" << DEG2RAD(pitch) << "\t" << DEG2RAD(yaw) << "\n";
    }
}

void gimbalAngleCallBack(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
    roll = msg->vector.y;
    pitch = msg->vector.x;
    yaw = msg->vector.z;

    print();
    save_txt();

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "read_sdk_gimbal");
    ros::NodeHandle nh;

    // ROS stuff
    start = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
    ros::Subscriber gimbal_angle_subscriber = nh.subscribe("dji_sdk/gimbal_angle", 10, &gimbalAngleCallBack);
    gimbal_pose.open("gimbal_pose.txt");
    while (ros::ok()) {
        ros::spin();
    }
}


