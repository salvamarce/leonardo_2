#include "ros/ros.h"
#include "aruco_manager.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_manager_test");
    ArucoManager am;

    return 0;
}