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
#define DEG2RAD(DEG) ((DEG) * ((M_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (M_PI))

///Parameters of RPA launcher
int resolution_x = 1920;
int resolution_y = 1080;
std::string string_rpa;
double aov_h;

/// Leitura dos pixels
int pixel_x, pixel_y;
int pixel_x_k, pixel_y_k;
/// Para resolucao de 800x600
int central_pixel_x = resolution_x / 2;
int central_pixel_y = resolution_y / 2;

/// darknet_ros/found_object parameter
int object_founded; //to save object detected number every iteration;
int object_id = 0;
int object_count;


/// Camera parameters
float Lx;
double dx = 1; //Distance to object
double GSD;

/// Parametros de controle para POS Y Pixel PID
// TODO: Ajustar parâmetros Kc e z0
double uy = 0, ux = 0;
double u_k_x = 0, u_k_y = 0;
double er_k_x = 0, er_k_y = 0;

/// Tempo de amostragem para malha de controle
int32_t Ts = 0.05; /// segundos
bool first_time = true;
double last_control, start_time;


/// Leitura Joint States
float roll, pitch, yaw;
float yaw_ik, pitch_ik, yaw_total, pitch_total;
float Kc = 0.8, z0 = 0.92;

using namespace std;
static std::ofstream inv_kinematic;
static std::ofstream control_pid;

class Inverse_Kinematic {
private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_joint_states;
    ros::Subscriber sub_boundingboxes;
    ros::Subscriber sub_found_object;

    ros::Subscriber sub_pitch;
    ros::Subscriber sub_yaw;

    std_msgs::Float64 msg_pitch;
    std_msgs::Float64 msg_yaw;

    ros::Publisher pub_pitch;
    ros::Publisher pub_yaw;


public:
    Inverse_Kinematic() {

        sub_pitch = nh_.subscribe("/" + GetParams::getRpaName() + "/gimbal_pitch/state", 10,
                                  &Inverse_Kinematic::pitch_value,
                                  this);
        sub_yaw = nh_.subscribe("/" + GetParams::getRpaName() + "/gimbal_yaw/state", 10,
                                &Inverse_Kinematic::yaw_value,
                                this);

        sub_found_object = nh_.subscribe("darknet_ros/found_object", 10, &Inverse_Kinematic::found_object, this);
        sub_boundingboxes = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &Inverse_Kinematic::read_px, this);

        pub_pitch = nh_.advertise<std_msgs::Float64>("/" + GetParams::getRpaName() + "/gimbal_pitch/command",
                                                     100);
        pub_yaw = nh_.advertise<std_msgs::Float64>("/" + GetParams::getRpaName() + "/gimbal_yaw/command", 1);


//        control_pid.open("pid_control.txt");
        inv_kinematic.open("inverse_kinematic.txt");
    }

    ~Inverse_Kinematic() {

    }

    static float round(float var) {
        float value = (int) (var * 10000 + 0.5);
        return (float) value / 10000;
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

        if (object_id == 4) {
            /// && (abs( pixel_x  - ((int) (msg->bounding_boxes[object_count].xmin-msg->bounding_boxes[object_count].xmax)/2 + (int) msg->bounding_boxes[object_count].xmin)) < 50 || first_time))
            ///object_id == 4 aeroplane || object_id == 32  sport_ball       || object_id == 49 orange              || object_id == 29 frisbee
            if (first_time) {
                pitch_total = pitch;
                yaw_total = yaw;
                first_time = false;
                last_control = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
                pixel_x = ((int) (msg->bounding_boxes[object_count].xmax - msg->bounding_boxes[object_count].xmin) / 2 +
                           (int) msg->bounding_boxes[object_count].xmin);
                pixel_y = ((int) (msg->bounding_boxes[object_count].ymax - msg->bounding_boxes[object_count].ymin) / 2 +
                           (int) msg->bounding_boxes[object_count].ymin);
                pixel_x_k = pixel_x;
                pixel_y_k = pixel_y;
                u_k_x = yaw;
                u_k_y = pitch;
                start_time = ros::Time::now().sec + ros::Time::now().nsec * 1e-9;
            }


            int xmin = msg->bounding_boxes[object_count].xmin;
            int xmax = msg->bounding_boxes[object_count].xmax;
            int ymin = msg->bounding_boxes[object_count].ymin;
            int ymax = msg->bounding_boxes[object_count].ymax;

            pixel_x = pixel_x * 0.6 + 0.4 * ((xmax - xmin) / 2 + xmin);
            pixel_y = pixel_y * 0.6 + 0.4 * ((ymax - ymin) / 2 + ymin);

//            if ((abs(pixel_x - pixel_x_k) < 80) && (abs(pixel_y - pixel_y_k) < 80)){
            pixel_x_k = pixel_x;
            pixel_y_k = pixel_y;
            print();
            if (inv_kinematic.is_open()) {
                inverse_kinematic();
            }
            if (control_pid.is_open()) {
                control();

            }
//            } else {
//                object_count++;
//                if (object_count > object_founded) {
//                    object_count = 0;
//                    cout << "\n\nNOT FOUND PROPER OBJECT" << endl;
//                    cout << "\033[2J\033[1;1H";     // clear terminal
//                }
//            }
        } else {
            object_count++;
            if (object_count > object_founded) {
                object_count = 0;
                cout << "\n\nNOT FOUND PROPER OBJECT" << endl;
                cout << "\033[2J\033[1;1H";     // clear terminal
            }
        }
        cout << "\033[2J\033[1;1H";     // clear terminal
    }

    void print() {
        cout << "Camera Resolution" << endl;
        cout << "\tResolution: " << resolution_x << " x " << resolution_y << "\tLx: " << Lx << "\tGSD: "
             << GSD << " m/px" << endl;

        cout << "Darknet detection" << endl;
        cout << "\tn_object: " << object_count << "\ttotal: " << object_founded << endl;
        cout << "\tobject_id: " << object_id << endl;
        cout << "PIXEL" << endl;
        cout << "\tPixel Pos: " << pixel_x << " x  " << pixel_y << endl;
        cout << "\tPixel REF: " << central_pixel_x << " x " << central_pixel_y << endl;
        cout << "\tPixel err: " << pixel_x - central_pixel_x << " x " << pixel_y - central_pixel_y << endl;

        cout << "Gimbal angles" << endl;
        cout << fixed << "\tr: " << RAD2DEG(round(roll)) << "\tp: " << RAD2DEG(round(pitch)) << "\ty: "
             << RAD2DEG( round(yaw))<< endl;
    }


    void inverse_kinematic() {
        double Zg = (float) (pixel_y - central_pixel_y) * GSD; //(px - px)*m/px
        double Yg = (float) (-pixel_x + central_pixel_x) * GSD; //(px - px)*m/px

        pitch_ik = asin(Zg / dx); //Zg/abs(Zg)*
        pitch_ik = round(pitch_ik);
        yaw_ik = asin(Yg / (dx * cos(pitch_ik))); //Yg/abs(Yg)*
        yaw_ik = round(yaw_ik);
        pitch_total = pitch_total * 0.9 + 0.1 * round(pitch + pitch_ik);
        yaw_total = yaw_total * 0.9 + 0.1 * round(yaw + yaw_ik);
        double dt = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec) - last_control;
        if (dt >= Ts) {
//            if (abs(-pixel_x + central_pixel_x) >= 20) {
            msg_yaw.data = yaw_total;
            pub_yaw.publish(msg_yaw);
//            }
//            if (abs((pixel_y - central_pixel_y)) >= 20) {
            msg_pitch.data = pitch_total;
            pub_pitch.publish(msg_pitch);
//            }
            if (inv_kinematic.is_open()) {
                double time = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec - start_time);
                inv_kinematic << time << "\t" << pitch << "\t" << yaw
                              << "\t" << pitch_total << "\t"
                              << yaw_total << "\t"
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
        double dt = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec) - last_control;
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
                double time = (ros::Time::now().nsec * 1e-9 + ros::Time::now().sec) - start_time;

                control_pid << time << "\t" << pitch << "\t" << yaw
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


    ros::init(argc, argv, "Gimbal_Control");
    Inverse_Kinematic ik;

    while (ros::Time::now().sec < 50) {
        cout << "\r" << "Waiting, time: " << ros::Time::now().sec << " seg" << std::flush;// << "\033[2J\033[1;1H";
    }
    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}





