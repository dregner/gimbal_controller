
#ifndef VANT3D_GAZEBO_GETPARAMS_H
#define VANT3D_GAZEBO_GETPARAMS_H

#include <ros/ros.h>

class GetParams {

private:
    static std::string string_var;
    static int resolution_x;
    static int resolution_y;
    static int aov_h;

public:
    static std::string getRpaName() ;
    static int getResolution_x();
    static int getResolution_y();
    static double getAov_h();
    static double getDistance_dx();
    static bool getUse_gimbal();



};


#endif //VANT3D_GAZEBO_GETPARAMS_H
