#include "ros/ros.h"
#include "axis_camera/Axis.h"

using namespace std;

struct PTZ_CONTROL{
   public:
      PTZ_CONTROL();

      void move_to();
      //Continuare a seconda delle necessità

   private:
      ros::NodeHandle _nh;
      ros::Subscriber _state_sub;
      ros::Publisher _ptz_pub;




};