#include "ros/ros.h"
#include "navigation.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation");
    Navigation nvg;
    nvg.setTestMode(true);
    nvg.run();

    return 0;
}