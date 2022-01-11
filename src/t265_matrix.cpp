#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "utils.h"
#include "Eigen/Dense"
#include "boost/thread.hpp"
#include <iostream>
#include <string>

using namespace std;

nav_msgs::Odometry current_odom;

void t265_cb(nav_msgs::Odometry odom){
   current_odom = odom;
}

void get_matrix(){
   string in;
   bool start, end;
   Eigen::Matrix4d T;
   Eigen::Vector3d start_pos;
   Eigen::Vector3d end_pos;
   Eigen::Vector4d start_quat;
   Eigen::Vector4d end_quat;

   ros::Rate r(10);

   T(3,0) = 0; T(3,1) = 0; 
   T(3,2) = 0; T(3,3) = 1;

   start = end = false;

   while(ros::ok()){
      cout << "-- Posizionare la camera rivolta verso l'asse X del drone.\n";
      cout << "-- Il frame della camera ha la X in avanti, la Y a sx e la Z verso l'alto \n";
      cout << "(nella trasformazione viene considerato il drone in terna ENU) \n";
      
      cout << "Inserire 1 quando si è pronti e tenere la camera ferma: \n";
      getline(cin, in);
      cout << "Tenere la camera ferma \n";

      if( in == "1" && !start ){
         Eigen::Vector3d sum_pos = Eigen::VectorXd::Zero(3);
         Eigen::Vector4d sum_quat = Eigen::VectorXd::Zero(4);
         ros::Rate rs(25);
         for(int i=0; i<50; i++){
            sum_pos << sum_pos(0) + current_odom.pose.pose.position.x,
                       sum_pos(1) + current_odom.pose.pose.position.y,
                       sum_pos(2) + current_odom.pose.pose.position.z;

            sum_quat << sum_quat(0) + current_odom.pose.pose.orientation.w,
                        sum_quat(1) + current_odom.pose.pose.orientation.x,
                        sum_quat(2) + current_odom.pose.pose.orientation.y,
                        sum_quat(3) + current_odom.pose.pose.orientation.z;
            rs.sleep();
         }
         start_pos = sum_pos/50.0;
         start_quat = sum_quat/50.0;

         cout << "Posizione acquisita \n";
         cout << "pos: " << start_pos.transpose() << endl;
         cout << "quat: " << start_quat.transpose() << endl;
         start = true;
         sleep(1.0);

         cout << "Posizionare la camera nella posizione di montaggio \n";
         cout << "Inserire 1 quando si è pronti e tenere la camera ferma: \n";
         getline(cin, in);
         cout << "Tenere la camera ferma  \n";

         if( in == "1" && !end ){
            Eigen::Vector3d sum_pos = Eigen::VectorXd::Zero(3);
            Eigen::Vector4d sum_quat = Eigen::VectorXd::Zero(4);
            ros::Rate rs(25);
            for(int i=0; i<50; i++){
               sum_pos << sum_pos(0) + current_odom.pose.pose.position.x,
                          sum_pos(1) + current_odom.pose.pose.position.y,
                          sum_pos(2) + current_odom.pose.pose.position.z;

               sum_quat << sum_quat(0) + current_odom.pose.pose.orientation.w,
                           sum_quat(1) + current_odom.pose.pose.orientation.x,
                           sum_quat(2) + current_odom.pose.pose.orientation.y,
                           sum_quat(3) + current_odom.pose.pose.orientation.z;
               rs.sleep();
            }
            end_pos = sum_pos/50.0;
            end_quat = sum_quat/50.0;

            cout << "Posizione acquisita \n";
            cout << "pos: " << end_pos.transpose() << endl;
            cout << "quat: " << end_quat.transpose() << endl;
            
            end = true;
         }

         if( start && end ){
            Eigen::Matrix3d rot;
            Eigen::Matrix3d start_mat, end_mat;
            Eigen::Vector3d pos;

            start_mat = utilities::QuatToMat(start_quat);
            end_mat = utilities::QuatToMat(end_quat);

            rot = end_mat * start_mat;
            pos = end_pos - start_pos;

            T(0,3) = pos(0);
            T(1,3) = pos(1);
            T(2,3) = pos(2);

            T(0,0) = rot(0,0); T(0,1) = rot(0,1); T(0,2) = rot(0,2);
            T(1,0) = rot(1,0); T(1,1) = rot(1,1); T(1,2) = rot(1,2);
            T(2,0) = rot(2,0); T(2,1) = rot(2,1); T(2,2) = rot(2,2);
            
            cout << "T: \n" << T << endl;
         }

         cout << "Vuoi ripetere la misura? (0 = No, 1 = Si) \n";
         getline(cin, in);

         if( in == "0" ){
            cout << "Fine \n";
            ros::shutdown();
         }
         else if( in == "1" ){
            start = false;
            end = false;
         }
         

      }
      r.sleep();
      
   }
}

int main(int argc, char** argv) {
   ros::init(argc, argv, "t265_matrix");
   ros::NodeHandle nh;
   
   ros::Subscriber odom_sub = nh.subscribe("/rst265/odom/sample", 2, t265_cb);

   boost::thread matrix_t( get_matrix);
   
   ros::spin();
   return 0;
}