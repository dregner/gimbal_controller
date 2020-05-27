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
double aov_h;

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
float Lx;
int focal_length = 30; //Focal length of camera len
double dx = 5; //Distence to object
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
int Ts = 30;
int countt = 0, counttt = 0, tout = 0;
bool first_time = true;

/// Leitura Joint States
float roll, pitch, yaw;
float yaw_ik, pitch_ik, yaw_total, pitch_total;


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

    std_msgs::Float64 msg_pitch;
    std_msgs::Float64 msg_yaw;

    ros::Publisher pub_pitch;
    ros::Publisher pub_yaw;


public:
    Inverse_Kinematic() {


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

        sub_found_object = nh_.subscribe("darknet_ros/found_object", 10, &Inverse_Kinematic::found_object, this);
        sub_boundingboxes = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &Inverse_Kinematic::read_px, this);

        pub_pitch = nh_.advertise<std_msgs::Float64>("/" + GetParams::getRpaName() + "/gimbal_pitch_position/command",
                                                     100);
        pub_yaw = nh_.advertise<std_msgs::Float64>("/" + GetParams::getRpaName() + "/gimbal_yaw_position/command", 1);


        states.open("inverse.txt");
    }

    ~Inverse_Kinematic() {

    }

    float round(float var) {
        float value = (int) (var * 1000 + 0.5);
        return (float) value / 1000;
    }

    void check_parameters() {
        resolution_y = GetParams::getResolution_y();
        resolution_x = GetParams::getResolution_x();
        central_pixel_x = resolution_x / 2;
        central_pixel_y = resolution_y / 2;
        dx = GetParams::getDistance_dx();
        aov_h = GetParams::getAov_h();
        focal_length = (int) (rad2deg * 2 * atan((float) resolution_x / 2 * pixel_dimenson / 1000)) / aov_h;
        Lx = 2 * tan(deg2rad * (float) aov_h / 2) * (float) dx;
        GSD = Lx / (float) resolution_x; // GSD from a gazebo environment where doesnt have a pixel dimension (m/px)
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
        if(first_time){
            pitch_total = pitch;
            yaw_total = yaw;
            first_time = false;
        }
    }

    void read_px(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
        object_id = msg->bounding_boxes[object_count].id;
        if (object_id == 49 || object_id == 32 || object_id == 29) {
            int xmin = msg->bounding_boxes[object_count].xmin;
            int xmax = msg->bounding_boxes[object_count].xmax;
            int ymin = msg->bounding_boxes[object_count].ymin;
            int ymax = msg->bounding_boxes[object_count].ymax;
            pixel_x = pixel_x * 0.95 + 0.05 * ((xmax - xmin) / 2 + xmin);
            pixel_y = pixel_y * 0.95 + 0.05 * ((ymax - ymin) / 2 + ymin);
            cout << "Camera Resolution" << endl;
            cout << "\tResolution: " << resolution_x << " x " << resolution_y << "\tLx: " << Lx << "\tGSD: "
                 << GSD << " m/px" << endl;
            cout << "\tCentral pixel: " << central_pixel_x << " x " << central_pixel_y << endl;
            cout << "Darknet detection" << endl;
            cout << "\tn_object: " << object_count << "\tpx: " << pixel_x << "\tpy: " << pixel_y << endl;
        } else {
            object_count++;
            if (object_count > object_founded) {
                object_count = 0;
                cout << "NOT FOUND PROPER OBJECT" << endl;
            }
        }
//        control();
        inverse_kinematic();


//        ROS_INFO("resolution: %i x %i - GSD: %f", resolution_x, resolution_y, GSD);
        cout << "Gimbal angles" << endl;
        cout << fixed << "\tr: " << round(roll) << "\tp: " << round(pitch) << "\ty: " << round(yaw) << endl;
        cout << "\033[2J\033[1;1H";     // clear terminal




    }


    void inverse_kinematic() {
        float Zg = (pixel_y - central_pixel_y) * GSD; //(px - px)*m/px
        float Yg = (-pixel_x + central_pixel_x) * GSD; //(px - px)*m/px
        cout << "XYZ Gimbal Inverse Kinematic" << endl;
        cout << "\tYg: " << Yg << "\tZg: " << Zg << endl;
        pitch_ik = asin(Zg / dx); //Zg/abs(Zg)*
        pitch_ik = round(pitch_ik);
        yaw_ik = asin(Yg / (dx * cos(pitch_ik))); //Yg/abs(Yg)*
        yaw_ik = round(yaw_ik);
        pitch_total = 0.8*pitch_total + 0.2*round(pitch + pitch_ik);
        yaw_total = 0.8*yaw_total + 0.2*round(yaw + yaw_ik);
        control_ik(yaw_total, pitch_total);
        cout << "Inverse" << endl;
        cout << "\tyaw ik: " << yaw_ik << "\tyaw desired: " << yaw_total << endl;
        cout << "\tpitch ik: " << pitch_ik << "\tpitch desired: " << pitch_total << endl;
        if (states.is_open() && counttt > Ts) {
            states << tout << "\t" << pitch << "\t" << yaw << "\t" << pitch_ik + pitch << "\t" << yaw_ik + yaw << "\t"
                   << Yg << "\t" << Zg << "\t"
                   << pixel_x << "\t" << pixel_y << "\t" << central_pixel_x - pixel_x << "\t"
                   << central_pixel_y - pixel_y
                   << "\n";
            counttt = 0;
        }
        counttt++;
    }

    void control_ik(float yaw, float pitch) {
        float ux, uy;
        countt++;
        tout++;
        if (countt > Ts) {
            msg_yaw.data = yaw;
            pub_yaw.publish(msg_yaw);
            msg_pitch.data = pitch;
            pub_pitch.publish(msg_pitch);
            countt = 0;
        }
    }


    void control() {
//        ros::Rate loop_rate(this->frequence);
        float er_x = central_pixel_x - pixel_x;
        float er_y = central_pixel_y - pixel_y;
        cout << "pixel error" << endl;
        cout << "\ter_x: " << er_x << "\ter_y: " << er_y << endl;
        er_x = er_x * GSD;
        er_y = er_y * GSD;
        if (countt > Ts) {
//            inverse_kinematic();
            control_x_position(er_x);
            control_y_position(er_y);
            countt = 0;
        }
//        loop_rate.sleep();
        er_k_y = er_y;
        er_k_x = er_x;
        if (states.is_open()) {
            states << tout << "\t" << pitch << "\t" << yaw << "\t" << u_k_x << "\t" << u_k_y << "\t"
                   << central_pixel_x - pixel_x << "\t"
                   << central_pixel_y - pixel_y << "\t" << pixel_x << "\t" << pixel_y << "\n";
        }
        countt++;
        tout++;


    }

    void control_x_position(float er_x) {

        float u;

        if (abs(er_x) > central_pixel_x * GSD) {
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
        if (abs(er_y) > central_pixel_y * GSD) {
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





