//
// Created by vant3d on 05/07/2020.
//

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <dji_sdk/Gimbal.h>
#include <iostream>

#define RAD2DEG(RAD) ((RAD) * (180.0) / (M_PI))
#define DEG2RAD(DEG) ((DEG) * (M_PI) / (180.0))

float roll, pitch, yaw;
int i = 0;
bool first_time = false;

using namespace std;

class Write_angle {
private:


    ros::NodeHandle nh;

    geometry_msgs::Vector3Stamped gimbal_angle;
    ros::Publisher gimbal_angle_pub;
    ros::Publisher gimbal_speed_pub;
    ros::Subscriber gimbal_angle_sub;


public:
    Write_angle() {

    gimbal_angle_pub = nh.advertise<dji_sdk::Gimbal>
            ("dji_sdk/gimbal_angle_cmd", 10);
    gimbal_speed_pub = nh.advertise<geometry_msgs::Vector3Stamped>
            ("dji_sdk/gimbal_speed_cmd", 10);
    gimbal_angle_sub = nh.subscribe<geometry_msgs::Vector3Stamped>
            ("dji_sdk/gimbal_angle", 10, &Write_angle::gimbalAngleCallback, this);



    }


    ~Write_angle() {}

    void set_angle(float roll, float pitch, float yaw) {
        dji_sdk::Gimbal gimbal_angle_data;
        gimbal_angle_data.mode |= 0 << 1; //Absulote = 1, Incremental = 0;
        gimbal_angle_data.mode |= 0 << 1;
        gimbal_angle_data.mode |= 0 << 2;
        gimbal_angle_data.mode |= 0 << 3;
        gimbal_angle_data.ts = 1; // duration seconds
        gimbal_angle_data.roll = DEG2RAD(roll);
        gimbal_angle_data.pitch = DEG2RAD(pitch);
        gimbal_angle_data.yaw = DEG2RAD(yaw);
        gimbal_angle_pub.publish(gimbal_angle_data);

        sleep(2);
    }

    void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        if (first_time == true) {
            set_speed(90, 90, 90); /// deg values
            first_time = false;
        }
        gimbal_angle = *msg;
        roll = gimbal_angle.vector.y;
        pitch = gimbal_angle.vector.x;
        yaw = gimbal_angle.vector.z;
        ROS_INFO("Gimbal angle: [ %f, %f, %f ] deg", roll, pitch, yaw);
        set_angle(0, 0, 0); /// deg values
    }

    void set_speed(int speed_r, int speed_p, int speed_y) {
        geometry_msgs::Vector3Stamped gimbalSpeed;
        gimbalSpeed.vector.y = DEG2RAD(speed_r);
        gimbalSpeed.vector.x = DEG2RAD(speed_p);
        gimbalSpeed.vector.z = DEG2RAD(speed_y);
        gimbal_speed_pub.publish(gimbalSpeed);
        sleep(1);
    }

};


int main(int argc, char **argv) {


    ros::init(argc, argv, "write_angle");
    ros::NodeHandle nh;

    Write_angle wa;


    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}



/*
int main(int argc, char **argv) {
    ros::init(argc, argv, "write_sdk_gimbal");
    ros::NodeHandle nh;

    // ROS stuff


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
}*/
