//
// Created by daniel.regner on 14/09/2020.
//

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ignition/math2/ignition/math/Pose3.hh>
#include <fstream>

#define RAD2DEG(RAD) ((RAD) * 180 / M_PI)

using namespace std;
static std::ofstream imu_pose;
float roll, pitch, yaw;
double start, last_control=0;
void save_txt() {
    if (imu_pose.is_open()) {
        double time_step = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec - start;
        imu_pose << time_step << "\t" << roll << "\t" << pitch << "\t" << yaw  << "\n";
    }
}

void callback(const sensor_msgs::Imu::ConstPtr &msg){
    ignition::math::Quaterniond rpy;
    rpy.Set(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    roll = rpy.Roll();
    pitch = rpy.Pitch();
    yaw = rpy.Yaw();
    cout << "R: " << RAD2DEG(rpy.Roll()) << endl;
    cout << "P: " << RAD2DEG(rpy.Pitch()) << endl;
    cout << "Y: " << RAD2DEG(rpy.Yaw()) << endl;
    cout << "OFFSET YAW:" << 90-RAD2DEG(rpy.Yaw()) << endl;
    cout << "\033[2J\033[1;1H";     // clear terminal
    double actual_time = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
    double dt = actual_time - last_control;
    if (dt >= 0.05) {
        save_txt();
        last_control = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec);
    }
}



int main(int argc, char **argv) {


    ros::init(argc, argv, "sdk_imu_listener");

    ros::NodeHandle nh;
    imu_pose.open("imu_pose.txt");
    start = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
    ros::Subscriber sub_imu = nh.subscribe("/dji_sdk/imu", 1000l, callback);

    ros::spin();
    return 0;
}