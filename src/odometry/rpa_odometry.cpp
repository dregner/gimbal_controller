#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <control_msgs/JointControllerState.h>
#include <GetParams.h>
#include <ignition/math/Pose3.hh>


/**
 * This tutorial demonstrates simple receipt of position and speed of the Evarobot over the ROS system.
 */

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */

float roll, pitch, yaw;
int Ts;
double last_control;
double RadToDeg = 180 / M_PI;
using namespace std;

static std::ofstream odometry;

class Listener {
private:
    ros::NodeHandle node;

    ros::Subscriber sub_roll;
    ros::Subscriber sub_pitch;
    ros::Subscriber sub_yaw;
    ros::Subscriber sub_o;
public:
    Listener() {

        if(GetParams::getUse_gimbal()) {
            sub_roll = node.subscribe("/" + GetParams::getRpaName() + "/gimbal_roll/state", 10,
                                      &Listener::roll_value,
                                      this);
            sub_pitch = node.subscribe("/" + GetParams::getRpaName() + "/gimbal_pitch/state", 10,
                                       &Listener::pitch_value,
                                       this);
            sub_yaw = node.subscribe("/" + GetParams::getRpaName() + "/gimbal_yaw/state", 10, &Listener::yaw_value,
                                     this);
        }
        sub_o = node.subscribe("/" + GetParams::getRpaName() + "/odometry_sensor/odometry",
                               10, &Listener::receivePos, this);


        odometry.open("odometry_ball.txt");

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

    void receivePos(const nav_msgs::Odometry::ConstPtr &msg) {

        ignition::math::Quaterniond rpy;
        rpy.Set(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z);

        cout << showpos << fixed << "Time:" << ros::Time::now().nsec * 1e-9 + ros::Time::now().sec << endl;
        cout << "Position" << endl;
        cout << setprecision(5) << "\tX:" << msg->pose.pose.position.x << endl;
        cout << setprecision(5) << "\tY:" << msg->pose.pose.position.y << endl;
        cout << setprecision(5) << "\tZ:" << msg->pose.pose.position.z << endl;
        cout << "RPA angles\t\tGimbal angles" << endl;
        cout << "\tRoll:" << rpy.Roll() << "\t\tRoll" << roll << endl;
        cout << "\tPitch:" << rpy.Pitch() << "\t\tPitch:" << pitch << endl;
        cout << "\tYaw:" << rpy.Yaw() << "\t\tYaw:" << yaw << endl;
        cout << "\033[2J\033[1;1H";     // clear terminal

        double actual_time = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
        double dt = actual_time - last_control;
        if (odometry.is_open() && dt > Ts) {
            odometry << ros::Time::now().nsec * 1e-9 + ros::Time::now().sec << "\t" << msg->pose.pose.position.x << "\t" << msg->pose.pose.position.y << "\t"
                     << msg->pose.pose.position.z << "\t" << rpy.Roll() << "\t"
                     << rpy.Pitch() << "\t" << rpy.Yaw() << "\t"
                     << roll << "\t" << pitch << "\t" << yaw << "\n";
            last_control = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
        }
    }
};






//    rpy.Set(q_w, q_x, q_y, q_z);
//    roll = rpy.Roll();
//    roll *= RadToDeg;
//    pitch = rpy.Pitch();
//    pitch *= RadToDeg;
//    yaw = rpy.Yaw();
//    yaw *= RadToDeg;

//    ROS_INFO("Seq: [%d]", msg->header.seq);
//    ROS_INFO("Position -> x: [%f], y: [%f], z: [%f]", pos_x, pos_y, pos_z);
//    ROS_INFO("Orientation -> r: [%f], p: [%f], y: [%f]", roll, pitch, yaw);


int main(int argc, char **argv) {
    ros::init(argc, argv, "ang_listener");

    Listener listener;
    while (ros::Time::now().sec < 50) {
        cout << "\r" << "Waiting, time: " << ros::Time::now().sec << " seg" << std::flush;// << "\033[2J\033[1;1H";
    }
    ros::spin();

    return 0;
}