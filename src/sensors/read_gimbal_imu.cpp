//
// Created by vant3d on 05/07/2020.
//

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <ignition/math2/ignition/math/Pose3.hh>
#include <fstream>
#include <boost/bind.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#define RAD2DEG(RAD) ((RAD) * (180.0) / (M_PI))
#define DEG2RAD(DEG) ((DEG) * (M_PI) / (180.0))

using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace std;

float g_roll, g_pitch, g_yaw;
float i_roll, i_pitch, i_yaw;
double start, last_control=0, actual_time;
int i = 0;
bool first_time = false;
static std::ofstream angles_pose;

void save_txt() {
    if (angles_pose.is_open()) {
        angles_pose << actual_time
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

void callback(const ImuConstPtr &pose_i,
            const Vector3StampedConstPtr &pose_g) {
    ignition::math::Quaterniond rpy;
    rpy.Set(pose_i->orientation.w, pose_i->orientation.x, pose_i->orientation.y, pose_i->orientation.z);
    i_roll = rpy.Roll();
    i_pitch = rpy.Pitch();
    i_yaw = rpy.Yaw();
    g_roll = pose_g->vector.y;
    g_pitch = pose_g->vector.x;
    g_yaw = pose_g->vector.z;
    print();
    actual_time = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
    double dt = actual_time - last_control;
    if (dt >= 0.05) {
        save_txt();
        last_control = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec);
    }
}


int main(int argc, char **argv) {


    ros::init(argc, argv, "read_imu_gimbal");
    ros::NodeHandle nh;
    message_filters::Subscriber<Imu> imu_pose(nh, "/dji_sdk/imu", 1);
    message_filters::Subscriber<Vector3Stamped> gimbal_pose(nh, "/dji_sdk/gimbal_angle", 1);

    angles_pose.open("angles_pose.txt");
    typedef message_filters::sync_policies::ApproximateTime<Imu, Vector3Stamped> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),imu_pose, gimbal_pose);
    sync.registerCallback(boost::bind(&callback, _1, _2));


    ros::spin();
    return 0;
}

