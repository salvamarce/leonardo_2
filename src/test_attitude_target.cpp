#include "ros/ros.h"
#include "leonardo_2/navigation.h"
#include "Eigen/Dense"
#include "boost/thread.hpp"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_att_target_main");
    ros::NodeHandle nh;
    ros::Rate r(10);
    Navigation *nvg = NULL;
    nvg = new Navigation();
    double w_roll, w_pitch, w_yaw, aT;
    w_roll = 0.0;
    w_pitch = 0.0;
    w_yaw = 0.0;
    aT = 0.5;
    while(ros::ok()){

        // nvg->setPointPublisherAtt(w_roll,w_pitch,w_yaw,aT);

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}