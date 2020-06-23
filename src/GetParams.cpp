
#include "GetParams.h"

std::string GetParams::getRpaName() {
    std::string string_var;
    if (ros::param::has("/rpa_name")) {
        ros::param::get("/rpa_name", string_var);
    } else {
        ros::param::param<std::string>("/rpa_name", string_var, "m600");
    }
    return string_var;
}


int GetParams::getResolution_y() {
    int resolution_y;
    if (ros::param::has("/camera_y_resolution")) {
        ros::param::get("/camera_y_resolution", resolution_y);
    } else {
        ros::param::param<std::int32_t>("/camera_y_resolution", resolution_y, 600);
    }
    return resolution_y;
}

int GetParams::getResolution_x() {
    int resolution_x;
    if (ros::param::has("/camera_x_resolution")) {
        ros::param::get("/camera_x_resolution", resolution_x);
    } else {
        ros::param::param<std::int32_t>("/camera_y_resolution", resolution_x, 800);
    }
    return resolution_x;
}

double GetParams::getAov_h() {
    double aov_h;
    if (ros::param::has("/aov_h")) {
        ros::param::get("/aov_h", aov_h);
    } else {
        ros::param::param<std::double_t>("/aov_h", aov_h, 45);
    }
    return aov_h;
}

double GetParams::getDistance_dx(){
    double dx;
    if (ros::param::has("/distance_riser")) {
        ros::param::get("/distance_riser", dx);
    } else {
        ros::param::param<std::double_t>("/distance_riser", dx, 5);
    }
    return dx;
}

bool GetParams::getUse_gimbal(){
    bool use_gimbal;
    if (ros::param::has("/use_gimbal")) {
        ros::param::get("/use_gimbal", use_gimbal);
    } else {
        ros::param::param<bool>("/use_gimbal", use_gimbal, false);
    }
}
