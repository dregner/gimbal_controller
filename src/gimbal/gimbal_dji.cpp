//
// Created by vant3d on 14/06/2020.
//
// System includes
#include "unistd.h"
#include <cstdint>
#include <iostream>
#include <fstream>

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
///Parameters of RPA launcher
int resolution_x = 1920;
int resolution_y = 1080;
std::string string_rpa;
double aov_h;
/// Para resolucao de 800x600
int central_pixel_x = resolution_x / 2;
int central_pixel_y = resolution_y / 2;
/// DX object
double dx = 1;
float Lx, GSD;

/// Position variables
float roll, pitch, yaw;
/// Darknet parameters
int object_founded, object_count, object_id;
/// Control
bool first_time = false;
int Ts = 0.05;

/// Inverse Kinematic CONTROLLER
float pitch_total, yaw_total;
double last_control;

/// PID CONTROLLER
double uy = 0, ux = 0;
double u_k_x = 0, u_k_y = 0;
double er_k_x = 0, er_k_y = 0;
float Kc = 0.8, z0 = 0.92;


/// Leitura dos pixels
int pixel_x, pixel_y;
int xmin_, xmin_k = xmin_;

using namespace std;
static std::ofstream inv_kinematic;
static std::ofstream control_pid;


class ControlGimbal_dji {
private:

    //ROS Node
    ros::NodeHandle nh;

    //Publisher and Subscribers
    geometry_msgs::Vector3Stamped gimbal_angle;
    geometry_msgs::Vector3Stamped gimbalSpeed;


    ros::Subscriber gimbal_angle_subscriber;
    ros::Publisher gimbal_angle_cmd_publisher;
    ros::Publisher gimbal_speed_cmd_publisher;
    ros::Subscriber sub_boundingboxes;
    ros::Subscriber sub_found_object;

    struct RotationAngle {
        float roll;
        float pitch;
        float yaw;
    } initialAngle, currentAngle;

    struct GimbalContainer {
        float roll = 0;
        float pitch = 0;
        float yaw = 0;
        int duration = 0;
        int isAbsolute = 0;
        bool yaw_cmd_ignore = false;
        bool pitch_cmd_ignore = false;
        bool roll_cmd_ignore = false;
        RotationAngle initialAngle;
        RotationAngle currentAngle;

        GimbalContainer(float roll = 0,
                        float pitch = 0,
                        float yaw = 0,
                        int duration = 0,
                        int isAbsolute = 0,
                        RotationAngle initialAngle = {},
                        RotationAngle currentAngle = {}) :
                roll(roll), pitch(pitch), yaw(yaw),
                duration(duration), isAbsolute(isAbsolute),
                initialAngle(initialAngle), currentAngle(currentAngle) {}
    } gimbal;


//        control_pid.open("pid_control.txt");
//    inv_kinematic.open("inverse_kinematic.txt");

public:
    ControlGimbal_dji() {
        // ROS stuff
        gimbal_angle_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>
                ("dji_sdk/gimbal_angle", 10, &ControlGimbal_dji::gimbalAngleCallback, this);
        sub_boundingboxes = nh.subscribe("darknet_ros/bounding_box", 10, &ControlGimbal_dji::ReadBb, this);
        sub_found_object = nh.subscribe("darknet_ros/found_object", 10, &ControlGimbal_dji::Found_obj, this);
        gimbal_angle_cmd_publisher = nh.advertise<dji_sdk::Gimbal>
                ("dji_sdk/gimbal_angle_cmd", 10);
        gimbal_speed_cmd_publisher = nh.advertise<geometry_msgs::Vector3Stamped>
                ("dji_sdk/gimbal_speed_cmd", 10);

    }

    ~ControlGimbal_dji() {}

    static float round(float var) {
        float value =(int) (var * 10000 + 0.5);
        return (float) value / 10000;
    }

    void Found_obj(const darknet_ros_msgs::ObjectCount::ConstPtr &msg) {
        object_founded = msg->count;

    }

    void save_txt() {
        if (inv_kinematic.is_open()) {
            inv_kinematic << ros::Time::now().nsec * 1e-9 + ros::Time::now().sec << "\t" << pitch << "\t" << yaw
                          << "\t" << pitch_total << "\t"
                          << yaw_total << "\t"
                          << pixel_x << "\t" << pixel_y << "\t" << central_pixel_x - pixel_x << "\t"
                          << central_pixel_y - pixel_y
                          << "\n";
        }
        if (control_pid.is_open()) {
            control_pid << ros::Time::now().nsec * 1e-9 + ros::Time::now().sec << "\t" << pitch << "\t" << yaw
                        << "\t" << u_k_x << "\t" << u_k_y << "\t"
                        << central_pixel_x - pixel_x << "\t"
                        << central_pixel_y - pixel_y << "\t" << pixel_x << "\t" << pixel_y << "\n";
        }
    }


    void setGimbalSeed(int speedRoll, int speedPitch, int speedYaw) {
        gimbalSpeed.vector.y = DEG2RAD(speedRoll); //deg
        gimbalSpeed.vector.x = DEG2RAD(speedPitch); //deg
        gimbalSpeed.vector.z = DEG2RAD(speedYaw); //deg
        gimbal_speed_cmd_publisher.publish(gimbalSpeed);
    }

    void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        gimbal_angle = *msg;
        currentAngle.roll = DEG2RAD(gimbal_angle.vector.y);
        currentAngle.pitch = DEG2RAD(gimbal_angle.vector.x);
        currentAngle.yaw = DEG2RAD(gimbal_angle.vector.z);
    }

    void ReadBb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
        object_id = msg->bounding_boxes[object_count].id;

        if ((object_id == 4)) {
            /// && (abs( pixel_x  - ((int) (msg->bounding_boxes[object_count].xmin-msg->bounding_boxes[object_count].xmax)/2 + (int) msg->bounding_boxes[object_count].xmin)) < 50 || first_time))
            ///object_id == 4 aeroplane || object_id == 32  sport_ball       || object_id == 49 orange              || object_id == 29 frisbee
            if (first_time) {
                initialAngle.roll = gimbal_angle.vector.y;
                initialAngle.pitch = gimbal_angle.vector.x;
                initialAngle.yaw = gimbal_angle.vector.z;
                pitch_total = gimbal_angle.vector.x;
                yaw_total = gimbal_angle.vector.z;
                first_time = false;
                last_control = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
                pixel_x = ((int) (msg->bounding_boxes[object_count].xmax - msg->bounding_boxes[object_count].xmin) / 2 +
                           (int) msg->bounding_boxes[object_count].xmin);
                pixel_y = ((int) (msg->bounding_boxes[object_count].ymax - msg->bounding_boxes[object_count].ymin) / 2 +
                           (int) msg->bounding_boxes[object_count].ymin);
                xmin_k = msg->bounding_boxes[object_count].xmin;
                u_k_x = yaw;
                u_k_y = pitch;
            }

//            if (abs(msg->bounding_boxes[object_count].xmin - xmin_k) < 100) {
            int xmin = msg->bounding_boxes[object_count].xmin;
            int xmax = msg->bounding_boxes[object_count].xmax;
            int ymin = msg->bounding_boxes[object_count].ymin;
            int ymax = msg->bounding_boxes[object_count].ymax;
            pixel_x = pixel_x * 0.6 + 0.4 * ((xmax - xmin) / 2 + xmin);
            pixel_y = pixel_y * 0.6 + 0.4 * ((ymax - ymin) / 2 + ymin);
            xmin_k = xmin;
            inverse_kinematic();
//            control();
//            }
        } else {
            object_count++;
            if (object_count > object_founded) { object_count = 0; }
        }
        cout << "\033[2J\033[1;1H";     // clear terminal
    }

    void inverse_kinematic() {
        float Zg = (float) (pixel_y - central_pixel_y) * GSD; //(px - px)*m/px
        float Yg = (float) (-pixel_x + central_pixel_x) * GSD; //(px - px)*m/px

        double pitch_ik = asin(Zg / dx); //Zg/abs(Zg)*
        pitch_ik = round(pitch_ik);
        double yaw_ik = asin(Yg / (dx * cos(pitch_ik))); //Yg/abs(Yg)*
        yaw_ik = round(yaw_ik);
        pitch_total = pitch_total * 0.9 + 0.1 * round(currentAngle.pitch + pitch_ik);
        yaw_total = yaw_total * 0.9 + 0.1 * round(currentAngle.yaw + yaw_ik);
        double actual_time = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
        double dt = actual_time - last_control;
//        cout << "\n" << dt << endl;
        if (dt >= Ts) {
            gimbal.initialAngle = initialAngle;
            gimbal.roll = 0;
            gimbal.pitch = pitch_total;
            gimbal.yaw = yaw_total;
            gimbal.isAbsolute = 1;
            doSetGimbalAngle(&gimbal);
            last_control = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
            save_txt();
        }
//        cout << "XYZ Gimbal Inverse Kinematic" << endl;
//        cout << "\tYg: " << Yg << "\tZg: " << Zg << endl;
        cout << "Inverse Kinematic Control:" << endl;
        cout << "\tyaw ik: " << yaw_ik << "\tyaw desired: " << yaw_total << endl;
        cout << "\tpitch ik: " << pitch_ik << "\tpitch desired: " << pitch_total << endl;

    }

    void control() {
        float er_x = (central_pixel_x - pixel_x) * GSD;
        float er_y = (central_pixel_y - pixel_y) * GSD;
        double actual_time = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
        double dt = actual_time - last_control;
        if (dt >= Ts) {
            if (abs(er_x) > central_pixel_x * GSD) {
                ux = u_k_x;
            } else {
                ux = (Kc * (er_x - z0 * er_k_x) + u_k_x);
            }
            if (abs(er_y) > central_pixel_y * GSD) {
                uy = u_k_y;
            } else {
                uy = (-Kc * (er_y - z0 * er_k_y) + u_k_y);
            }
            gimbal.initialAngle = initialAngle;
            gimbal.roll = 0;
            gimbal.pitch = uy;
            gimbal.yaw = ux;
            gimbal.isAbsolute = 1;
            doSetGimbalAngle(&gimbal);
            u_k_x = ux;
            u_k_y = uy;
            save_txt();
            last_control = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec);
        }
        cout << "PID Control: " << endl;
        cout << "\tUx: " << u_k_x << "\tUy: " << u_k_y << endl;

        er_k_y = er_y;
        er_k_x = er_x;

    }

    void doSetGimbalAngle(ControlGimbal_dji::GimbalContainer *gimbal) {
        dji_sdk::Gimbal gimbal_angle_data;
        gimbal_angle_data.mode |= 0;
        gimbal_angle_data.mode |= gimbal->isAbsolute;
        gimbal_angle_data.mode |= gimbal->yaw_cmd_ignore << 1;
        gimbal_angle_data.mode |= gimbal->roll_cmd_ignore << 2;
        gimbal_angle_data.mode |= gimbal->pitch_cmd_ignore << 3;
        gimbal_angle_data.ts = gimbal->duration;
        gimbal_angle_data.roll = gimbal->roll;
        gimbal_angle_data.pitch = gimbal->pitch;
        gimbal_angle_data.yaw = gimbal->yaw;

        gimbal_angle_cmd_publisher.publish(gimbal_angle_data);
        // Give time for gimbal to sync
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "gimbal_track");
    ControlGimbal_dji control;

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}