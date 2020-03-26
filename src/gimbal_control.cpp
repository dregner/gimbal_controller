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

/// Leitura Joint States
float roll, pitch, yaw;

//float erro_yaw = 0;
float old_data = 0;

/// Parametros de controle Junta ROLL
// TODO: Ajustar parâmetros Kc e z0
const float Kc_roll = -.8;
float erro_roll = 0;
float erro_roll_k = 0;
const float z0_roll = .85;
float u_roll = 0;
float u_roll_k = u_roll;




/// Tempo de amostragem para malha de controle
int Ts = 20;
int countt = 0;


using namespace std;

class Gimbal_Control {
private:
    ros::NodeHandle nh_;

    ros::Subscriber sub_joint_states;
    ros::Subscriber sub_pixel_x;
    ros::Subscriber sub_pixel_y;

    std_msgs::Float64 msg_roll;
    std_msgs::Float64 msg_pitch;
    std_msgs::Float64 msg_yaw;


    ros::Publisher pub_roll;
    ros::Publisher pub_pitch;
    ros::Publisher pub_yaw;

    ignition::math::Quaterniond rpy;


public:
    Gimbal_Control(){

        sub_pixel_x = nh_.subscribe("/pixel_x", 10, &Gimbal_Control::read_px_x, this);
        sub_pixel_y = nh_.subscribe("/pixel_y", 10, &Gimbal_Control::read_px_y, this);
        sub_joint_states = nh_.subscribe("/gimbal/joint_state", 100, &Gimbal_Control::read_JS, this);


//        image_sub_ = it_.subscribe("/m600/camera_labmetro/image_raw", 1, &Gimbal_Control::publish, this);

        pub_roll = nh_.advertise<std_msgs::Float64>("/m600/gimbal_joint_roll_position_controller/command", 100);
        pub_pitch = nh_.advertise<std_msgs::Float64>("/m600/gimbal_joint_pitch_position_controller/command", 100);
        pub_yaw = nh_.advertise<std_msgs::Float64>("/m600/gimbal_joint_yaw_position_controller/command", 1);
    }

    ~Gimbal_Control() {

    }

    void read_px_x(const std_msgs::Int16Ptr &msg){
        pixel_x = msg->data;
    }

    void read_px_y(const std_msgs::Int16Ptr &msg){
        pixel_y = msg->data;
    }

    void read_JS(const sensor_msgs::JointStatePtr &msg) {


        pitch = msg->position[0];
        roll = msg->position[1];
        yaw = msg->position[2];

    }

//        if(countt>Ts) {
//            this->control_yaw();
//            this->control_roll();
//            this->control_pitch();
//            countt = 0;
//        }

//        loop_rate.sleep();
//        countt++;
//
//        erro_yaw_k = erro_yaw;
//        erro_roll_k = erro_roll;
//        erro_pitch_k = erro_pitch;

    void pixel_error_x(){


    }


    void control_roll() {
        u_roll = (Kc_roll * (-erro_roll_k + z0_roll * erro_roll) + u_roll_k);
        msg_roll.data = u_roll;
        pub_roll.publish(msg_roll);
        ROS_INFO("Control: [%f]", u_roll);
        u_roll_k = u_roll;
        countt = 0;
    }

};
int main(int argc, char **argv) {


    ros::init(argc, argv, "Gimbal_Control");

    Gimbal_Control gb;
    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}





