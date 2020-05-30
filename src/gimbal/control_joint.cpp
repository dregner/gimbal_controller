//
// Created by daniel regner on 25/03/2020.
//
#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
//#include <gimbal_controller/BoundingBox.h>
#include <sensor_msgs/JointState.h>
#include <ignition/math/Pose3.hh>
#include <image_transport/image_transport.h>


/**
 * @authors Digo Henrique e Daniel Regner.
 * @copyright Labmetro UFSC
 * @brief Plugin de controle a partir da posição lida da IMU para setar juntas do Gimbal
 */

/**
 *
 * @param msg :
 * @return Vazio
 */
/// Leitura dos pixels
int pixel_x, pixel_y;
/// Para resolucao de 800x600
int central_pixel_x = 400;
int central_pixel_y = 300;


/* Parametros de Controle */
int frequence = 60;
/// Parametros de controle para POS Y Pixel
// TODO: Ajustar parâmetros Kc e z0
float error_px_y = 0, error_px_x = 0;
float u_px_y = 0, u_px_x = 0;
float u_px_y_k = u_px_y, u_px_x_k = u_px_x;
float u_k_x = 0, u_k_y = 0;
float er_k_x = 0, er_k_y = 0;

/// Tempo de amostragem para malha de controle
int Ts = 10;
int countt = 0, tout = 0;
int histerese_error_x = 5;
int histerese_error_y = 5;
float GDS = 0.002695;
bool first_time = true;

/// Leitura Joint States
float roll, pitch, yaw;


using namespace std;
static std::ofstream states;

class Gimbal_Control {
private:


    int frequence = 10;

    ros::NodeHandle nh_;

    ros::Subscriber sub_joint_states;
    ros::Subscriber sub_pixel;
    ros::Subscriber sub_pixel_y;

    std_msgs::Float64 msg_roll;
    std_msgs::Float64 msg_pitch;
    std_msgs::Float64 msg_yaw;


    ros::Publisher pub_roll;
    ros::Publisher pub_pitch;
    ros::Publisher pub_yaw;


public:
    Gimbal_Control() {

        sub_pixel = nh_.subscribe("/boundingbox", 10, &Gimbal_Control::read_px, this);
        sub_joint_states = nh_.subscribe("/gimbal/joint_states", 100, &Gimbal_Control::read_JS, this);

        pub_roll = nh_.advertise<std_msgs::Float64>("/gimbal/roll_position_controller/command", 100);
        pub_pitch = nh_.advertise<std_msgs::Float64>("/gimbal/pitch_position_controller/command", 100);
        pub_yaw = nh_.advertise<std_msgs::Float64>("/gimbal/yaw_position_controller/command", 1);


        states.open("states.txt");
    }

    ~Gimbal_Control() {

    }

    void read_px(const gimbal_controller::BoundingBoxConstPtr &msg){

        int xmin = msg->xmin;
        int xmax = msg->xmax;
        int ymin = msg->ymin;
        int ymax = msg->ymax;
        pixel_x = (xmax-xmin)/2 + xmin;
        pixel_y = (ymax-ymin)/2 + ymin;
        ROS_INFO("X: %i Y: %i", pixel_x, pixel_y);

    }

    void read_JS(const sensor_msgs::JointStatePtr &msg) {


        pitch = msg->position[0];
        roll = msg->position[1];
        yaw = msg->position[2];
        //ROS_INFO("rpy: %f\t%f\t%f", roll, pitch, yaw);
        if(first_time == true){
            u_k_x = yaw;
            u_k_y = pitch;
            first_time = false;
        }
        control();


    }

    void control() {
//        ros::Rate loop_rate(this->frequence);
        float er_x = central_pixel_x - pixel_x;
        float er_y = central_pixel_y - pixel_y;
        ROS_INFO("er_x: %f", er_x);
        ROS_INFO("er_y: %f", er_y);
        er_x = er_x * GDS;
        er_y = er_y * GDS;
        if (countt > Ts) {
            control_x_position(er_x);
            control_y_position(er_y);
            countt = 0;
        }
//        loop_rate.sleep();
        er_k_y = er_y;
        er_k_x = er_x;
        countt++;
        tout++;

        if (states.is_open() && tout < 2000) {
            states << tout << "\t" << yaw << "\t" << u_k_x << "\t" << er_x / GDS
                   << "\t" << pitch << "\t" << u_k_x << "\t" << er_y / GDS << "\n";
        }
    }

    void control_x_position(float er_x) {

        float u;

        if (abs(er_x) > central_pixel_x * GDS) {
            u = u_k_x;
        } else {
            u = (0.10 * (er_k_x - 0.75 * er_x) + u_k_x);
        }
        msg_yaw.data = u;
        pub_yaw.publish(msg_yaw);
//        ROS_INFO("u: %f", u);
        u_k_x = u;
    }

    void control_y_position(float er_y) {
        float u;
        if (abs(er_y) > central_pixel_y * GDS) {
            u = u_k_y;
        } else {
            u = (-0.15 * (er_k_y - 0.75 * er_y) + u_k_y);
        }
        msg_pitch.data = u;
        pub_pitch.publish(msg_pitch);
//        ROS_INFO("u: %f", u);
        u_k_y = u;
    }

};

int main(int argc, char **argv) {


    ros::init(argc, argv, "Gimbal_Control");

    Gimbal_Control gb;

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}





