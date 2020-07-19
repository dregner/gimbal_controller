//
// Created by daniel regner on 25/03/2020.
//
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>



int xmin, xmax, ymin, ymax;
int header, object_found;

using namespace std;


class Read_bb {
private:


    int frequence = 10;

    ros::NodeHandle nh_;

    ros::Subscriber sub_boundingboxes;
    ros::Subscriber sub_found_object;


public:
    Read_bb() {

        sub_boundingboxes = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &Read_bb::reading, this);
        sub_found_object = nh_.subscribe("/darknet_ros/found_object", 10, &Read_bb::found_object, this);
//



//        states.open("states.txt");
    }

    ~Read_bb() {

    }

    void found_object(const darknet_ros_msgs::ObjectCount::ConstPtr &msg) {
        object_found = msg->count;
    }

    /// Sporting ball ID=32
    void reading(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
        cout << "Bouding Boxes (header):" << msg->header << endl;
        cout << "Bouding Boxes (image_header):" << msg->image_header << endl;
        for (int i = 0; i < object_found; i++) {
            cout << "Bouding Boxes (Class):" << msg->bounding_boxes[i].Class << endl;
            cout << "Bouding Boxes (ID):" << msg->bounding_boxes[i].id << endl;
            cout << "Bouding Boxes (xmin):" << msg->bounding_boxes[i].xmin << endl;
            cout << "Bouding Boxes (xmax):" << msg->bounding_boxes[i].xmax << endl;
            cout << "Bouding Boxes (ymin):" << msg->bounding_boxes[i].ymin << endl;
            cout << "Bouding Boxes (ymax):" << msg->bounding_boxes[i].ymax << endl;
        }
        cout << "\033[2J\033[1;1H";     // clear terminal

    }
};

int main(int argc, char **argv) {


    ros::init(argc, argv, "read_bb");

    Read_bb ik;

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}





//
// Created by vant3d on 16/05/2020.
//

