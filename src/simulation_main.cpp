#include "ros/ros.h"
#include "aruco_manager.h"
#include "navigation.h"
#include "Eigen/Dense"
#include "boost/thread.hpp"

using namespace std;

int main(int argc, char** argv) {
   ros::init(argc, argv, "simulation_main_leonardo");
   
	ArucoManager aruco(true);

	aruco.run();

	ros::spin();
   
   return 0;
}
