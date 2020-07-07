//
// Created by vant3d on 05/07/2020.
//

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <dji_sdk/Gimbal.h>
#include <iostream>

#define RAD2DEG(RAD) ((RAD) * (180.0) / (M_PI))

float roll, pitch, yaw;
int inputValue_y = 0, inputValue_r = 0, inputValue_p = 0;
bool mode = 0;


using namespace std;

geometry_msgs::Vector3Stamped gimbal_angle;
ros::Publisher gimbal_angle_pub;
ros::Subscriber gimbal_angle_subscriber;


void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
    gimbal_angle = *msg;
    roll = gimbal_angle.vector.y;
    pitch = gimbal_angle.vector.x;
    yaw = gimbal_angle.vector.z;

    ros::spinOnce();
}
void set_angle(){
    dji_sdk::Gimbal gimbal_angle_data;
    gimbal_angle_data.mode |= 0;
    gimbal_angle_data.mode |= mode; //Absulote = 1, Incremental = 0;
    gimbal_angle_data.mode |= 0 << 1;
    gimbal_angle_data.mode |= 0 << 2;
    gimbal_angle_data.mode |= 0 << 3;
    gimbal_angle_data.ts = 1; // duration seconds
    gimbal_angle_data.roll = inputValue_r;
    gimbal_angle_data.pitch = inputValue_p;
    gimbal_angle_data.yaw = inputValue_y;
    gimbal_angle_pub.publish(gimbal_angle_data);
    sleep(2);
}

void gimbal_angle_func(int input) {
    cout << "Value: " << input << endl;

    ros::spinOnce();
    ROS_INFO("Initial Gimbal rotation angle: [ %f, %f, %f ] deg", roll, pitch, yaw);

    set_angle();

    ros::spinOnce();
    ROS_INFO("Initial Gimbal rotation angle: [ %f, %f, %f ] deg", roll, pitch, yaw);
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "write_sdk_gimbal");
    ros::NodeHandle nh;

    // ROS stuff
    gimbal_angle_pub = nh.advertise<dji_sdk::Gimbal>("dji_sdk/gimbal_angle_cmd", 10);
    gimbal_angle_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>
            ("dji_sdk/gimbal_angle", 10, &gimbalAngleCallback);
    // Display interactive prompt
    cout << "| Available commands:" << endl;
    cout << "\t[a] Change moode" << endl;
    cout << "\t[b] Change roll position" << endl;
    cout << "\t[c] Change pitch position" << endl;
    cout << "\t[d] Change yaw position" << endl;
    char inputChar;
    cin >> inputChar;

    switch (inputChar) {
        case 'a':
            cout << "0 - Incremental or 1 - Absolute" << endl;
            cin >> mode;
            gimbal_angle_func(mode);
            break;
        case 'b':
            cout << "Roll value (deg): ";
            cin >> inputValue_r;
            gimbal_angle_func(inputValue_r);
            break;
        case 'c':
            cout << "Pitch value (deg): ";
            cin >> inputValue_p;
            gimbal_angle_func(inputValue_p);
            break;
        case 'd':
            cout << "Yaw value (deg): ";
            cin >> inputValue_y;
            gimbal_angle_func(inputValue_y);
            break;
        default:
            break;
    }
}