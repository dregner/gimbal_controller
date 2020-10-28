//
// Created by vant3d on 14/06/2020.
//
// System includes
#include "unistd.h"
//#include <cstdint>
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
int resolution_x = 1280;
int resolution_y = 720;
/// Para resolucao de 800x600
int central_pixel_x = resolution_x / 2;
int central_pixel_y = resolution_y / 2;
/// DX object
double dx = 1;
float GSD = 0.000208;

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
int start;

/// Leitura dos pixels
int pixel_x, pixel_y;
int xmin_, xmin_k = xmin_;
float yaw_offset = 0;


using namespace std;
static std::ofstream inv_kinematic;


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


public:
    ControlGimbal_dji() {
        // ROS stuff
        gimbal_angle_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>
                ("dji_sdk/gimbal_angle", 10, &ControlGimbal_dji::gimbalAngleCallback, this);
        sub_boundingboxes = nh.subscribe("/darknet_ros/bounding_boxes", 10, &ControlGimbal_dji::ReadBb, this);
        sub_found_object = nh.subscribe("darknet_ros/found_object", 10, &ControlGimbal_dji::Found_obj, this);
        gimbal_angle_cmd_publisher = nh.advertise<dji_sdk::Gimbal>
                ("dji_sdk/gimbal_angle_cmd", 10);
        gimbal_speed_cmd_publisher = nh.advertise<geometry_msgs::Vector3Stamped>
                ("dji_sdk/gimbal_speed_cmd", 10);
        inv_kinematic.open("ik_tracK_control.txt");
    }

    ~ControlGimbal_dji() {}

    static float round(float var) {
        float value = (int) (var * 10000 + 0.5);
        return (float) value / 10000;
    }

    void Found_obj(const darknet_ros_msgs::ObjectCount::ConstPtr &msg) {
        object_founded = msg->count;

    }

    void save_txt() {
        if (inv_kinematic.is_open()) {
	    float  time_now = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec) - start;
            inv_kinematic << time_now << "\t" << pitch << "\t" << yaw
                          << "\t" << pitch_total << "\t"
                          << yaw_total << "\t"
                          << pixel_x << "\t" << pixel_y << "\t" << central_pixel_x - pixel_x << "\t"
                          << central_pixel_y - pixel_y
                          << "\n";
        }
    }


    void setGimbalSpeed(int speedRoll, int speedPitch, int speedYaw) {
        gimbalSpeed.vector.y = DEG2RAD(speedRoll); //deg
        gimbalSpeed.vector.x = DEG2RAD(speedPitch); //deg
        gimbalSpeed.vector.z = DEG2RAD(speedYaw); //deg
        gimbal_speed_cmd_publisher.publish(gimbalSpeed);
    }

    void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        gimbal_angle = *msg;
        roll = DEG2RAD(gimbal_angle.vector.y);
        pitch = DEG2RAD(gimbal_angle.vector.x);
        yaw = DEG2RAD(yaw_offset - gimbal_angle.vector.z);
    }

    void ReadBb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
        object_id = msg->bounding_boxes[object_count].id;


        if ((object_id == 56)) {
            // 4 aeroplane; 56 chair; 67  cell_phone; 0 person; 66 keyboard
            if (first_time) {
                first_time = false;
                last_control = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
                pixel_x = ((int) (msg->bounding_boxes[object_count].xmax - msg->bounding_boxes[object_count].xmin) / 2 +
                           (int) msg->bounding_boxes[object_count].xmin);
                pixel_y = ((int) (msg->bounding_boxes[object_count].ymax - msg->bounding_boxes[object_count].ymin) / 2 +
                           (int) msg->bounding_boxes[object_count].ymin);
            }

            int xmin = msg->bounding_boxes[object_count].xmin;
            int xmax = msg->bounding_boxes[object_count].xmax;
            int ymin = msg->bounding_boxes[object_count].ymin;
            int ymax = msg->bounding_boxes[object_count].ymax;
            pixel_x = pixel_x * 0.6 + 0.4 * ((xmax - xmin) / 2 + xmin);
            pixel_y = pixel_y * 0.6 + 0.4 * ((ymax - ymin) / 2 + ymin);
            cout << central_pixel_x << " x " << central_pixel_y << endl;
            cout << "\tpx_x: " << pixel_x << "\tpx_y: " << pixel_y << endl;

            inverse_kinematic();

        } else {
            object_count++;
            if (object_count > object_founded) { object_count = 0; }
        }
        cout << "\033[2J\033[1;1H";     // clear terminal
    }

    void inverse_kinematic() {
        float Zg = (float) (-pixel_y + central_pixel_y) * GSD; //(px - px)*m/px
        float Yg = (float) (-pixel_x + central_pixel_x) * GSD; //(px - px)*m/px

        double yaw_ik = asin(-Yg / dx); // Yg = -sin(yaw_ik)*dx
        yaw_ik = round(yaw_ik);
        double pitch_ik = asin(Zg / (dx * cos(yaw_ik))); //Zg = cos(yaw_ik)*sen(pitch_ik)*dx

        pitch_ik = round(pitch_ik);

        //pitch_total = pitch + pitch_ik;
        //yaw_total = yaw + yaw_ik;
        double actual_time = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
        double dt = actual_time - last_control;
//        cout << "\n" << dt << endl;
        if (dt >= Ts) {
            doSetGimbalAngle(0, pitch_ik, yaw_ik, 1,0);
            last_control = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec);
            save_txt();
	    pitch_total = pitch_ik+pitch;
	    yaw_total = yaw_ik+	yaw;
        }

        cout << "Inverse Kinematic Control:" << endl;
        cout << "\tYg: " << Yg << "\tZg: " << Zg << endl;
        cout << "\tpx_x: " << pixel_x << "\tpx_y: " << pixel_y << endl;
        cout << "\tsp_x: " << central_pixel_x << "\tsp_y: " << central_pixel_y << endl;
        cout << "\ter_x: " << Yg / GSD << "\ter_y: " << Zg / GSD << endl;
        cout << "\tyaw ik: " << yaw_ik << "\tyaw desired: " << yaw_total << endl;
        cout << "\tpitch ik: " << pitch_ik << "\tpitch desired: " << pitch_total << endl;

    }


    void doSetGimbalAngle(float roll, float pitch, float yaw, int duration, bool mode) {

        dji_sdk::Gimbal gimbal_angle_data;
        gimbal_angle_data.mode |= mode << 0; // 1 - absolute, 0 - incremental
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
    if(argc > 0){

        yaw_offset = std::strtof(argv[0], NULL);
    }
    cout << "offset: " << yaw_offset << endl;
    ros::init(argc, argv, "gimbal_track");
    ControlGimbal_dji control;
    cout << "Initialize Control" << endl;
    control.doSetGimbalAngle(0, 0, 0, 1, 1);
    control.setGimbalSpeed(90, 90, 90);
    cout << "Waiting to def speed 90 deg/sec" << endl;
    sleep(2);
    cout << "Done" << endl;
    start = ros::Time::now().sec;
    ros::spinOnce();
    cout << "Init angle: " << RAD2DEG(roll) << ", " << RAD2DEG(pitch) << ", " << RAD2DEG(yaw) << endl;

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
