#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float32MultiArray.h"
//---mavros_msgs
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandTOL.h"
#include <mavros_msgs/PositionTarget.h>
//---
#include "utils.h"
#include "Eigen/Dense"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
//---

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "planner_spline.h"

#include <mutex>          // std::mutex
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Imu.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>

//Socket functions
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>

#pragma once

using namespace Eigen;
using namespace std;


// --- Socket and TF stuff ---
struct Pose3D {
	float x;
	float y;
	float z;
	float qw;
	float qx;
	float qy;
	float qz;
};

struct Poses3D {
	Pose3D bl_a;
	Pose3D o_a;
	Pose3D bl_o;
};

//Creazione socket in SCRITTURA
inline int create_socket(char* dest, int port, int *sock) {
	struct sockaddr_in temp; 
	struct hostent *h;
	int error;

	temp.sin_family=AF_INET;
	temp.sin_port=htons(port);
	h=gethostbyname(dest);

	if (h==0) {
		printf("Gethostbyname fallito\n");
		exit(1);
	}

	bcopy(h->h_addr,&temp.sin_addr,h->h_length);
	*sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	error=connect(*sock, (struct sockaddr*) &temp, sizeof(temp));
	return error;
}

// --- Navigation class ---

class Navigation{
	public:
		Navigation ();

		//Check if the setpoint is reached (already used in move functions)
      bool arrived( const Eigen::Vector3d pos_sp, const double yaw_sp );
		bool arrived( const Eigen::Vector3d pos_sp );
		bool arrived( const double yaw_sp );
		
		//Perform a takeoff at a desired altitude with desired velocity
		void takeoff( const double altitude, double vel = 0.0 ); 

		//Land at a desired altitude with desired velocity
		void land( double altitude = -1.0, const double vel = 0.0 );
		
		/* Move to a desired position with a desired velocity
		all theese functions uses world(arena) coordinate */		
		void moveTo( const Eigen::Vector3d dest, const double vel = 0.0 );
		void moveToWithYaw( const Eigen::Vector3d dest, const double yaw , const double vel = 0.0 );
		void moveToWps( const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> dest, const double vel = 0.0 );
		void moveToWpsWithYaw( const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> dest, const Eigen::Ref<Eigen::VectorXd> yaw , const double vel = 0.0 );
		
		//Rotate with a desired yaw and velocity
		void rotate( const double angle, double vel = 0.0); 

		//Activate or disable trajectory generator
		void activateTrajectoryGenerator( const bool act) { _act_traj_gen = act;}

		//Interrupt movements
		void interruptAll() { _interrupt = true; }
		void resetInterrupt() { _interrupt = false; }
		//Get functions
		const bool getTakeoff() {return _take_off;}
		const bool getLand() {return !_take_off;}
		const Eigen::Vector4d getWorldPos() { return _world_pos; }
		const Eigen::Vector4d getWorldPosOdom() { return _world_pos_odom; }
		const double getYaw() { return _mes_yaw; }
		const double getYawOdom() { return _mes_yaw_odom; }
		const Eigen::Vector4d getWorldQuatOdom() { return _world_quat_odom; }
		const Eigen::Vector4d getWorldQuat() { return _world_quat; }
		const Eigen::Matrix4d getWorldTransform() { return _H_odom_arena; }
		const bool getInterrupt() { return _interrupt; }
		const bool getLocalizationStatus() { return _localization_status; }
		const bool setLocalizationStatus(const bool status ) { _localization_status = status; }

		//Set functions
		//Set the trasformation matrix form odom to world
		void setWorldTransform(const Eigen::Ref<Eigen::Matrix<double, 4, 4>> new_H_odom_Arena);

		//Publisher to the mavros topic, in odom coordinates
		void setPointPublisher();

		//TF functions
		void tf_broadcast_pose_arena();
		void tf_broadcast_pose_odom();
		void tf_broadcast_odom_arena();
		void rviz_sp_publisher();
		void tf_broadcast_poses();

	private:
		void pose_cb ( geometry_msgs::PoseStamped msg );
		void mavros_state_cb( mavros_msgs::State mstate);
		void tof_reads_cb ( std_msgs::Float32MultiArray values );
		void trajectoryGenerator(const Eigen::Vector3d pos, const double yaw, const double vel);
		void trajectoryGeneratorWps(const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> pos, const Eigen::Ref<Eigen::VectorXd> yaw , const double vel);
		void load_tof_angles();

		ros::NodeHandle _nh;
		ros::Publisher _setpoint_pub;
		ros::Publisher _setpoint_bag_pub;
		ros::Subscriber _pose_sub;
		ros::Subscriber _mavros_state_sub;
		tf::TransformBroadcaster _broadcaster;

		string _setpoint_topic;
		string _mavros_state_topic;
		string _pose_topic;

		// --- Desired state
		Vector3d _pos_sp;
		Vector3d _vel_sp;
		Vector3d _des_pos_sp;
		double _yaw_sp;
		double _des_yaw_sp;
		double _sp_rate;
		
		// --- Drone state ---
		Vector4d _world_pos;
		Vector4d _world_pos_odom;
		Matrix4d _H_odom_arena;
		Vector4d _world_quat;
		Vector4d _world_quat_odom;
		float _mes_yaw;
		float _mes_yaw_odom;
      mavros_msgs::State _mstate;
      bool _take_off;
		bool _act_traj_gen;
		bool _localization_status;
		bool _first_local_pos;

		// --- Clients services
		ros::ServiceClient _arming_client;
		ros::ServiceClient _set_mode_client;
		ros::ServiceClient _land_client;

      // --- Trajectory planner ---
      CARTESIAN_PLANNER *_trajectory = NULL;
		Eigen::Vector3d _setpoint;
		Eigen::Vector3d _old_setpoint;
		double _cruise_vel;
		double _cruise_acc;
		double _traj_rate;
		bool _interrupt;

		// --- Thresholds ---
		double _pos_threshold;
		double _yaw_threshold;
		double _height_threshold;
		double _max_height;
		double _drone_height;

		// --- TOF variables ---
		// std_msgs::Float32MultiArray 
		Eigen::Matrix<double,8,1> _tof_angles;
		double _critical_distance;
		bool _critical_state;

};
