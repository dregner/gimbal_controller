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


/// Camera parameters
float deg2rad = M_PI / 180;
float Lx;
double dx = 5; //Distence to object
double GSD;


/// Parametros de controle para POS Y Pixel PID
// TODO: Ajustar parâmetros Kc e z0
double uy = 0, ux = 0;
double u_k_x = 0, u_k_y = 0;
double er_k_x = 0, er_k_y = 0;

/// Tempo de amostragem para malha de controle
int32_t Ts = 0.05; /// segundos
bool first_time = true;
double dt, last_control;


/// Leitura Joint States
float roll, pitch, yaw;
float yaw_ik, pitch_ik, yaw_total, pitch_total;


using namespace std;
static std::ofstream inv_kinematic;
static std::ofstream control_pid;

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


//        control_pid.open("control_pid.txt");
//        inv_kinematic.open("inverse.txt");
    }

    ~Inverse_Kinematic() {

    }

    static float round(float var) {
        float value = (int) (var * 1000 + 0.5);
        return (float) value / 1000;
    }

    static void check_parameters() {
        resolution_y = GetParams::getResolution_y();
        resolution_x = GetParams::getResolution_x();
        central_pixel_x = resolution_x / 2;
        central_pixel_y = resolution_y / 2;
        dx = GetParams::getDistance_dx();
        aov_h = GetParams::getAov_h();
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

    }

    void read_px(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
        object_id = msg->bounding_boxes[object_count].id;
        if (object_id == 49 || object_id == 32 || object_id == 29) {
            int xmin = msg->bounding_boxes[object_count].xmin;
            int xmax = msg->bounding_boxes[object_count].xmax;
            int ymin = msg->bounding_boxes[object_count].ymin;
            int ymax = msg->bounding_boxes[object_count].ymax;
            if (first_time) {
                pitch_total = pitch;
                yaw_total = yaw;
                first_time = false;
                last_control = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
                pixel_x = ((xmax - xmin) / 2 + xmin);
                pixel_y = ((ymax - ymin) / 2 + ymin);
                u_k_x = yaw;
                u_k_y = pitch;
            }
            pixel_x = pixel_x * 0.7 + 0.3 * ((xmax - xmin) / 2 + xmin);
            pixel_y = pixel_y * 0.7 + 0.3 * ((ymax - ymin) / 2 + ymin);
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
                cout << "\n\nNOT FOUND PROPER OBJECT" << endl;
                cout << "\033[2J\033[1;1H";     // clear terminal
            }
        }
        control();



//        inverse_kinematic();


//        ROS_INFO("resolution: %i x %i - GSD: %f", resolution_x, resolution_y, GSD);
        cout << "Gimbal angles" << endl;
        cout << fixed << "\tr: " << round(roll) << "\tp: " << round(pitch) << "\ty: " << round(yaw) << endl;
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
        cout << "\n" << dt << endl;
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
        cout << "XYZ Gimbal Inverse Kinematic" << endl;
        cout << "\tYg: " << Yg << "\tZg: " << Zg << endl;
        cout << "Inverse" << endl;
        cout << "\tyaw ik: " << yaw_ik << "\tyaw desired: " << yaw_total << endl;
        cout << "\tpitch ik: " << pitch_ik << "\tpitch desired: " << pitch_total << endl;

    }


    void control() {
        float er_x = (central_pixel_x - pixel_x)*GSD;
        float er_y = (central_pixel_y - pixel_y)*GSD;
        double actual_time = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
        dt = actual_time - last_control;
        if (dt >= Ts) {


            if (abs(er_x) > central_pixel_x * GSD) {
                 ux = u_k_x;
            } else {
                 ux = (0.05 * (er_k_x - 0.75 * er_x) + u_k_x);
            }
            if (abs(er_y) > central_pixel_y * GSD) {
                 uy = u_k_y;
            } else {
                 uy = (-0.05 * (er_k_y - 0.75 * er_y) + u_k_y);
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
        cout << "pixel error" << endl;
        cout << "\ter_x: " << er_x << "\ter_y: " << er_y << endl;
//        cout << "Control: " << endl;
        cout << "\tUx: " << u_k_x << "\tUy: " << u_k_y << endl;

        er_k_y = er_y;
        er_k_x = er_x;

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





