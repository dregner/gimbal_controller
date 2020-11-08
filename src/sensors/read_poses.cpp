//
// Created by vant3d on 05/07/2020.
//

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <ignition/math2/ignition/math/Pose3.hh>


#define RAD2DEG(RAD) ((RAD) * (180.0) / (M_PI))
#define DEG2RAD(DEG) ((DEG) * (M_PI) / (180.0))

float g_roll, g_pitch, g_yaw;
float i_roll, i_pitch, i_yaw;
int i = 0;
bool first_time = false;
double start, last_control=0, actual_time;
static std::ofstream angles_pose;
using namespace std;

class ReadAngles {
private:


    ros::NodeHandle nh;

    geometry_msgs::Vector3Stamped gimbal_angle;
    ros::Subscriber imu_angle_sub;
    ros::Subscriber gimbal_angle_sub;


public:
    ReadAngles() {

        gimbal_angle_sub = nh.subscribe<geometry_msgs::Vector3Stamped>
                ("dji_sdk/gimbal_angle", 10, &ReadAngles::gimbalAngleCallback, this);
        imu_angle_sub = nh.subscribe<sensor_msgs::Imu>
                ("dji_sdk/imu",10, &ReadAngles::read_imu, this);
        angles_pose.open("angles_pose.txt");


    }


    ~ReadAngles() {}

    void read_imu(const sensor_msgs::Imu::ConstPtr &msg){
        msg->orientation.x;
        ignition::math::Quaterniond rpy;
        rpy.Set(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        i_roll = rpy.Roll();
        i_pitch = rpy.Pitch();
        i_yaw = rpy.Yaw();
        actual_time = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
        double dt = actual_time - last_control;
        if (dt >= 0.05) {
            save_txt();
            last_control = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec);
        }

    }

    void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        gimbal_angle = *msg;
        g_roll = gimbal_angle.vector.y;
        g_pitch = gimbal_angle.vector.x;
        g_yaw = gimbal_angle.vector.z;
        print();
    }

    void save_txt() {
        if (angles_pose.is_open()) {
            double time_step = actual_time - start;
            angles_pose << time_step
                        << "\t" << DEG2RAD(g_roll) << "\t" << DEG2RAD(g_pitch) << "\t" << DEG2RAD(g_yaw)
                        << "\t" << i_roll << "\t" << i_pitch << "\t" << i_yaw  << "\n";
        }
    }
    void print(){
        cout << "IMU" << endl;
        cout << "\tR: " << RAD2DEG(i_roll) << endl;
        cout << "\tP: " << RAD2DEG(i_pitch) << endl;
        cout << "\tY: " << RAD2DEG(i_yaw) << endl;
        cout << "Gimbal" << endl;
        cout << "\tR: " << g_roll << endl;
        cout << "\tP: " << g_pitch << endl;
        cout << "\tY: " << g_yaw << endl;

    }
};


int main(int argc, char **argv) {


    ros::init(argc, argv, "read_angles");
    ros::NodeHandle nh;

    ReadAngles gimbal_imu;
    start = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;


    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
