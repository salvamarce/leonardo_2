#include "ros/ros.h"
#include "aruco_manager.h"
#include "navigation.h"
#include "Eigen/Dense"
#include "boost/thread.hpp"

using namespace std;

void routine();

int main(int argc, char** argv) {
   ros::init(argc, argv, "main_leonardo");
   ArucoManager arm;

   arm.run();
   
   return 0;
}
