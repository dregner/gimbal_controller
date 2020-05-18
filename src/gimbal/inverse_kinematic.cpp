//
// Created by daniel regner on 25/03/2020.
//
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <GetParams.h>

#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>

#include <std_msgs/Int64.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>


///Parameters of RPA launcher
int resolution_x = 1920;
int resolution_y = 1080;
std::string string_rpa;
int aov_h;

/// Leitura dos pixels
int pixel_x, pixel_y;
/// Para resolucao de 800x600
int central_pixel_x = resolution_x / 2;
int central_pixel_y = resolution_y / 2;

/// darknet_ros/found_object parameter
int object_founded; //to save object detected number every iteration;
int object_id = 0;
int object_count;


/// X5S specs
float deg2rad = M_PI / 180;
float rad2deg = 180 / M_PI;
float pixel_dimenson = 3.4; //Pixel dimension of X5S
float L;
int focal_length = 30; //Focal length of camera len
int dx = 5; //Distence to object
float GSD = dx * pixel_dimenson / focal_length;


/* Parametros de Controle */
int frequence = 60;
/// Parametros de controle para POS Y Pixel
// TODO: Ajustar parâmetros Kc e z0
float error_px_y = 0, error_px_x = 0;
float u_px_y = 0, u_px_x = 0;
float u_k_x = 0, u_k_y = 0;
float er_k_x = 0, er_k_y = 0;

/// Tempo de amostragem para malha de controle
int Ts = 20;
int countt = 0, tout = 0;
bool first_time = true;

/// Leitura Joint States
float roll, pitch, yaw;
float yaw_ik, pitch_ik;


using namespace std;
static std::ofstream states;

class Inverse_Kinematic {
private:

    ros::NodeHandle nh_;

    ros::Subscriber sub_joint_states;
    ros::Subscriber sub_boundingboxes;
    ros::Subscriber sub_found_object;

    ros::Subscriber sub_roll;
    ros::Subscriber sub_pitch;
    ros::Subscriber sub_yaw;


    std_msgs::Float64 msg_roll;
    std_msgs::Float64 msg_pitch;
    std_msgs::Float64 msg_yaw;


    ros::Publisher pub_roll;
    ros::Publisher pub_pitch;
    ros::Publisher pub_yaw;


public:
    Inverse_Kinematic() {

        sub_found_object = nh_.subscribe("darknet_ros/found_object", 10, &Inverse_Kinematic::found_object, this);
        sub_boundingboxes = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &Inverse_Kinematic::read_px, this);

        check_parameters();

        sub_roll = nh_.subscribe("/" + GetParams::getRpaName() + "/gimbal_roll_position/state", 10,
                                 &Inverse_Kinematic::roll_value,
                                 this);
        sub_pitch = nh_.subscribe("/" + GetParams::getRpaName() + "/gimbal_pitch_position/state", 10,
                                  &Inverse_Kinematic::pitch_value,
                                  this);
        sub_yaw = nh_.subscribe("/" + GetParams::getRpaName() + "/gimbal_yaw_position/state", 10,
                                &Inverse_Kinematic::yaw_value,
                                this);

        pub_pitch = nh_.advertise<std_msgs::Float64>("/" + GetParams::getRpaName() + "/gimbal_pitch_position/command",
                                                     100);
        pub_yaw = nh_.advertise<std_msgs::Float64>("/" + GetParams::getRpaName() + "/gimbal_yaw_position/command", 1);


//        states.open("states.txt");
    }

    ~Inverse_Kinematic() {

    }

    void check_parameters() {
        resolution_y = GetParams::getResolution_y();
        resolution_x = GetParams::getResolution_x();
        aov_h = GetParams::getAov_h();
        focal_length = (rad2deg * 2 * atan(resolution_x / 2 * pixel_dimenson / 1000)) / aov_h;
        L = 2*tan(deg2rad*aov_h/2)*dx;
//        GSD = dx * pixel_dimenson / focal_length;
        GSD = 1000*L/resolution_x;
    }

    void roll_value(const control_msgs::JointControllerState::ConstPtr &msg) {
        roll = msg->process_value;

    }

    void pitch_value(const control_msgs::JointControllerState::ConstPtr &msg) {
        pitch = msg->process_value;
    }

    void yaw_value(const control_msgs::JointControllerState::ConstPtr &msg) {
        yaw = msg->process_value;
    }

    void found_object(const darknet_ros_msgs::ObjectCount::ConstPtr &msg) {
        object_founded = msg->count;
    }

    void read_px(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
        object_id = msg->bounding_boxes[object_count].id;
        if (object_id == 49 || object_id == 32) {
            int xmin = msg->bounding_boxes[object_count].xmin;
            int xmax = msg->bounding_boxes[object_count].xmax;
            int ymin = msg->bounding_boxes[object_count].ymin;
            int ymax = msg->bounding_boxes[object_count].ymax;
            pixel_x = pixel_x*0.95 + 0.05*((xmax - xmin) / 2 + xmin);
            pixel_y = pixel_y*0.95 + 0.05*((ymax - ymin) / 2 + ymin);
            cout << "Camera Resolution" << endl;
            cout << "\tResolution: " << resolution_x << " x " << resolution_y << "\tLx: " << L << "\tGSD: " << GSD/1000 << " m/px" << endl;
            cout  << "Darknet detection" << endl;
            cout << "\tn_object: " << object_count << "\tpx: " << pixel_x << "\tpy: " << pixel_y << endl;
        } else {
            object_count++;
            if (object_count > object_founded) {
                object_count = 0;
                cout << "NO FOUNDED PROPER OBJECT" << endl;
            }
        }
//        control();
        inverse_kinematic();


//        ROS_INFO("resolution: %i x %i - GSD: %f", resolution_x, resolution_y, GSD);
        cout << "Gimbal angles" << endl;
        cout << fixed  << "\tr: " << round(roll) << "\tp: " << round(pitch) << "\ty: " << round(yaw) << endl;
        cout << "\033[2J\033[1;1H";     // clear terminal

    }

    float round(float var)
    {
        // 37.66666 * 100 =3766.66
        // 3766.66 + .5 =3767.16    for rounding off value
        // then type cast to int so value is 3767
        // then divided by 100 so the value converted into 37.67
        float value = (int)(var * 1000+ .5);
        return (float)value / 1000;
    }


    void inverse_kinematic() {
        float Zg = (pixel_y - central_pixel_y) * GSD / 1000; //(px - px)*1e-3*m/px/1e3
        float Yg = (-pixel_x + central_pixel_x) * GSD / 1000; //(px - px)*1e-3*m/px1e3
        cout << "XYZ Gimbal Inverse Kinematic" <<endl;
        cout << "\tYg: " << Yg <<"\tZg: " <<Zg << endl;
        pitch_ik = asin(Zg / dx); //Zg/abs(Zg)*
        pitch_ik = round(pitch_ik);
        yaw_ik = asin(Yg / (dx * cos(pitch_ik))); //Yg/abs(Yg)*
        yaw_ik = round(yaw_ik);
        float pitch_total = round(pitch + pitch_ik);
        float yaw_total = round(yaw + yaw_ik);
//        control_ik(yaw_ik, pitch_ik);
        cout << "Inverse" << endl;
        cout << "\tyaw ik: " << yaw_ik << "\tyaw desired: " << yaw_total << endl;
        cout << "\tpitch ik: " << pitch_ik << "\tpitch desired: " << pitch_total <<endl;

    }

    void control_ik(float yaw, float pitch) {
        float ux, uy;
        ux = u_k_x + yaw;
        uy = u_k_y + pitch;
        msg_yaw.data = ux;
        pub_yaw.publish(msg_yaw);
        msg_pitch.data = uy;
        pub_pitch.publish(msg_pitch);
        u_k_x = ux;
        u_k_y = uy;
    }


    void control() {
//        ros::Rate loop_rate(this->frequence);
        float er_x = central_pixel_x - pixel_x;
        float er_y = central_pixel_y - pixel_y;
        cout << "pixel error" << endl;
        cout << "\ter_x: " <<er_x <<"\ter_y: " << er_y<< endl;
        er_x = er_x * GSD / 1000;
        er_y = er_y * GSD / 1000;
        if (countt > Ts) {
//            inverse_kinematic();
            control_x_position(er_x);
            control_y_position(er_y);
            countt = 0;
        }
//        loop_rate.sleep();
        er_k_y = er_y;
        er_k_x = er_x;
        countt++;
        tout++;

//        if (states.is_open() && tout < 2000) {
//            states << tout << "\t" << yaw << "\t" << u_k_x << "\t" << er_x / GDS
//                   << "\t" << pitch << "\t" << u_k_x << "\t" << er_y / GDS << "\n";
//        }
    }

    void control_x_position(float er_x) {

        float u;

        if (abs(er_x) > central_pixel_x * GSD / 1000) {
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
        if (abs(er_y) > central_pixel_y * GSD / 1000) {
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


    ros::init(argc, argv, "Inverse_Kinematic");

    Inverse_Kinematic ik;

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}





