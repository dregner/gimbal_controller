//
// Created by vant3d on 14/06/2020.
//
// System includes
#include "unistd.h"
#include <cstdint>
#include <iostream>
#include <GetParams.h>

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
int dx;
float Lx, GSD;

using namespace std;

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

    float roll, pitch, yaw;
    int object_founded, object_count, object_id;
    bool first_time = false;
    float pitch_total, yaw_total, u_k_x, u_k_y;
    double last_control;



    /// Leitura dos pixels
    int pixel_x, pixel_y;
    int xmin_, xmin_k = xmin_;



public:
    ControlGimbal_dji() {
        check_parameters();
//        DJI::OSDK::Gimbal::SpeedData gimbalSpeed;
//        DJI::OSDK::Gimbal::AngleData gimbalAngle;
//        DJI::OSDK::Vehicle vehicle;


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

    static void check_parameters() {
        resolution_y = GetParams::getResolution_y();
        resolution_x = GetParams::getResolution_x();
        central_pixel_x = resolution_x / 2;
        central_pixel_y = resolution_y / 2;
        dx = GetParams::getDistance_dx();
        aov_h = GetParams::getAov_h();
        Lx = 2 * tan(DEG2RAD(aov_h)/ 2) * (float) dx;
        GSD = Lx / (float) resolution_x; // GSD from a gazebo environment where doesnt have a pixel dimension (m/px)
    }

    void Found_obj(const darknet_ros_msgs::ObjectCount::ConstPtr &msg){
        object_founded = msg->count;

    }
    void setGimbalSeed(int speedRoll, int speedPitch, int speedYaw) {
        gimbalSpeed.vector.y = DEG2RAD(speedRoll); //deg
        gimbalSpeed.vector.x = DEG2RAD(speedPitch); //deg
        gimbalSpeed.vector.z = DEG2RAD(speedYaw); //deg
        gimbal_speed_cmd_publisher.publish(gimbalSpeed);
    }

    void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        gimbal_angle = *msg;
        roll = gimbal_angle.vector.y;
        pitch = gimbal_angle.vector.x;
        yaw = gimbal_angle.vector.z;
    }

    void ReadBb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){
        object_id = msg->bounding_boxes[object_count].id;

        if ((object_id == 4)) {
            /// && (abs( pixel_x  - ((int) (msg->bounding_boxes[object_count].xmin-msg->bounding_boxes[object_count].xmax)/2 + (int) msg->bounding_boxes[object_count].xmin)) < 50 || first_time))
            ///object_id == 4 aeroplane || object_id == 32  sport_ball       || object_id == 49 orange              || object_id == 29 frisbee
            if (first_time) {
                pitch_total = pitch;
                yaw_total = yaw;
                first_time = false;
                last_control = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
                pixel_x = ((int) (msg->bounding_boxes[object_count].xmin - msg->bounding_boxes[object_count].xmax) / 2 +
                           (int) msg->bounding_boxes[object_count].xmin);
                pixel_y = ((int) (msg->bounding_boxes[object_count].ymin - msg->bounding_boxes[object_count].ymax) / 2 +
                           (int) msg->bounding_boxes[object_count].ymin);
                xmin_k = msg->bounding_boxes[object_count].xmin;
                u_k_x = yaw;
                u_k_y = pitch;
            }

            if (abs(msg->bounding_boxes[object_count].xmin - xmin_k) < 100) {
                int xmin = msg->bounding_boxes[object_count].xmin;
                xmin_k = xmin;
                int xmax = msg->bounding_boxes[object_count].xmax;
                int ymin = msg->bounding_boxes[object_count].ymin;
                int ymax = msg->bounding_boxes[object_count].ymax;

                pixel_x = pixel_x * 0.6 + 0.4 * ((xmax - xmin) / 2 + xmin);
                pixel_y = pixel_y * 0.6 + 0.4 * ((ymax - ymin) / 2 + ymin);

                print();
                inverse_kinematic();
            }


        } else {
            object_count++;
            if (object_count > object_founded) {
                object_count = 0;
                cout << "\n\nNOT FOUND PROPER OBJECT" << endl;
                cout << "\033[2J\033[1;1H";     // clear terminal
            }
        }


//        control();

        cout << "\033[2J\033[1;1H";     // clear terminal
    }

    void inverse_kinematic() {
        double Zg = (float) (pixel_y - central_pixel_y) * GSD; //(px - px)*m/px
        double Yg = (float) (-pixel_x + central_pixel_x) * GSD; //(px - px)*m/px

        pitch_ik = asin(Zg / dx); //Zg/abs(Zg)*
        pitch_ik = round(pitch_ik);
        yaw_ik = asin(Yg / (dx * cos(pitch_ik))); //Yg/abs(Yg)*
        yaw_ik = round(yaw_ik);
        pitch_total = round(pitch + pitch_ik);
        yaw_total = round(yaw + yaw_ik);
        double actual_time = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
        dt = actual_time - last_control;
//        cout << "\n" << dt << endl;
        if (dt >= Ts) {
            msg_yaw.data = yaw_total;
            pub_yaw.publish(msg_yaw);
            msg_pitch.data = pitch_total;
            pub_pitch.publish(msg_pitch);
            if (inv_kinematic.is_open()) {
                inv_kinematic << ros::Time::now().nsec * 1e-9 + ros::Time::now().sec << "\t" << pitch << "\t" << yaw
                              << "\t" << pitch_ik + pitch << "\t"
                              << yaw_ik + yaw << "\t"
                              << Yg << "\t" << Zg << "\t"
                              << pixel_x << "\t" << pixel_y << "\t" << central_pixel_x - pixel_x << "\t"
                              << central_pixel_y - pixel_y
                              << "\n";
            }
            last_control = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;

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
        dt = actual_time - last_control;
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
            msg_pitch.data = uy;
            pub_pitch.publish(msg_pitch);
            msg_yaw.data = ux;
            pub_yaw.publish(msg_yaw);
            u_k_x = ux;
            u_k_y = uy;


            if (control_pid.is_open()) {
                control_pid << ros::Time::now().nsec * 1e-9 + ros::Time::now().sec << "\t" << pitch << "\t" << yaw
                            << "\t" << u_k_x << "\t" << u_k_y << "\t"
                            << central_pixel_x - pixel_x << "\t"
                            << central_pixel_y - pixel_y << "\t" << pixel_x << "\t" << pixel_y << "\n";
            }
            last_control = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;

        }
        cout << "PID Control: " << endl;
        cout << "\tUx: " << u_k_x << "\tUy: " << u_k_y << endl;

        er_k_y = er_y;
        er_k_x = er_x;

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