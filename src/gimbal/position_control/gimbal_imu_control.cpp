/*
 * This algorithm presents a control to reject RPA movements over gimbal
 * structure and maintain stable, rejecting disturbances and keep focused on a specify point of view
 */

// System includes
#include "unistd.h"
//#include <cstdint>
#include <iostream>
#include <fstream>

// DJI OSDK includes
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_gimbal.hpp>
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/Gimbal.h>

// ROS includes
#include <ros/ros.h>
//Sensor reads
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <ignition/math2/ignition/math/Pose3.hh>


#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))


/// Position variables
float roll, pitch, yaw;
float rpa_roll, rpa_pitch, rpa_yaw;

/// Control
bool first_time = false;
int Ts = 0.2;
float last_control;


/// Angle reference
float ref_yaw = 0;
float ref_pitch = 0;
/// PID CONTROLLER
double u_yaw = 0, u_pitch = 0;
double u_k_pitch = 0, u_k_yaw = 0;
double er_k_pitch = 0, er_k_yaw = 0;
double er_pitch, er_yaw;
float Kc = 0.08, z0 = 0.92;


using namespace std;
static std::ofstream imu_control;


class Gimbal_control_imu {
private:

    //ROS Node
    ros::NodeHandle nh;

    //Publisher and Subscribers
    geometry_msgs::Vector3Stamped gimbal_angle;
    geometry_msgs::Vector3Stamped gimbalSpeed;

    /// Gimbal topics
    ros::Subscriber gimbal_angle_subscriber;
    ros::Publisher gimbal_angle_cmd_publisher;
    ros::Publisher gimbal_speed_cmd_publisher;
    /// IMU topic
    ros::Subscriber sdk_imu_subscriber;


public:
    Gimbal_control_imu() {
        // ROS stuff
        gimbal_angle_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>
                ("dji_sdk/gimbal_angle", 10, &Gimbal_control_imu::gimbalAngleCallback, this);
        gimbal_angle_cmd_publisher = nh.advertise<dji_sdk::Gimbal>
                ("dji_sdk/gimbal_angle_cmd", 10);
        gimbal_speed_cmd_publisher = nh.advertise<geometry_msgs::Vector3Stamped>
                ("dji_sdk/gimbal_speed_cmd", 10);
        sdk_imu_subscriber = nh.subscribe<sensor_msgs::Imu>("dji_sdk/imu", 10, &Gimbal_control_imu::read_imu, this);

    }

    ~Gimbal_control_imu() {}


    void setGimbalSpeed(int speedRoll, int speedPitch, int speedYaw) {
        gimbalSpeed.vector.y = DEG2RAD(speedRoll); //deg
        gimbalSpeed.vector.x = DEG2RAD(speedPitch); //deg
        gimbalSpeed.vector.z = DEG2RAD(speedYaw); //deg
        gimbal_speed_cmd_publisher.publish(gimbalSpeed);
    }

    void read_imu(const sensor_msgs::Imu::ConstPtr &msg){
        ignition::math::Quaterniond rpa_rpy;
        rpa_rpy.Set(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        rpa_roll = RAD2DEG(rpa_rpy.Roll());
        rpa_pitch = RAD2DEG(rpa_rpy.Pitch());
        rpa_yaw = RAD2DEG(rpa_rpy.Yaw());
    }

    void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        gimbal_angle = *msg;
        roll = DEG2RAD(gimbal_angle.vector.y);
        pitch = DEG2RAD(gimbal_angle.vector.x);
        yaw = DEG2RAD( gimbal_angle.vector.z);
        if (first_time) {
            setGimbalSpeed(90, 90, 90);
            sleep(2);
            er_pitch = er_yaw = 0;
        }
        control();
    }


    void control() {
        er_pitch = ref_pitch - pitch;
        er_yaw = ref_yaw - yaw;

        double actual_time = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
        double dt = actual_time - last_control;
        if (dt >= Ts) {
            u_pitch = (Kc * (er_pitch - z0 * er_k_pitch) + u_k_pitch);
            u_yaw = (Kc * (er_yaw - z0 * er_k_yaw) + u_k_yaw);

            doSetGimbalAngle(0, u_pitch, u_yaw, 1);
            u_k_pitch = u_pitch;
            u_k_yaw = u_yaw;
            last_control = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec);
        }
        er_k_yaw = er_yaw;
        er_k_pitch = er_pitch;

    }

    void doSetGimbalAngle(float roll, float pitch, float yaw, int duration) {
        dji_sdk::Gimbal gimbal_angle_data;
        gimbal_angle_data.mode |= 1 << 0; // 1 - absolute, 0 - incremental
        gimbal_angle_data.mode |= 0 << 1; // yaw_cmd_ignore
        gimbal_angle_data.mode |= 0 << 2; // roll_cmd_ignore
        gimbal_angle_data.mode |= 0 << 3; // pitch_cmd_ignore
        gimbal_angle_data.ts = duration;
        gimbal_angle_data.roll = roll;
        gimbal_angle_data.pitch = pitch;
        gimbal_angle_data.yaw = yaw;

        gimbal_angle_cmd_publisher.publish(gimbal_angle_data);
        // Give time for gimbal to sync
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "gimbal_track");
    Gimbal_control_imu control;
    cout << "Initialize Control" << endl;
    control.doSetGimbalAngle(0, 0, 0, 10);
    ros::spinOnce();
    cout << "Init angle: " << RAD2DEG(roll) << ", " << RAD2DEG(pitch) << ", " << RAD2DEG(yaw) << endl;

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
