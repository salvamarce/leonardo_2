#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
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

#include "planner_spline.h"

#include <mutex>          // std::mutex
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Imu.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>

#pragma once

using namespace Eigen;
using namespace std;

class Navigation{
	public:
		Navigation ();

		void run();

		//Check if the setpoint is reached
      bool arrived( const Eigen::Vector3d pos_sp, const double yaw_sp );
		bool arrived( const Eigen::Vector3d pos_sp );
		bool arrived( const double yaw_sp );
		
		//Perform a takeoff at a desired altitude with desired velocity
		void takeoff( const double altitude, double vel = 0.0 ); 
		
		//Move to a desired position with a desired velocity
		void moveToWpsWithYaw( const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> dest, const Eigen::Ref<Eigen::VectorXd> yaw , const double vel = 0.0 );
		void moveToWps( const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> dest, const double vel = 0.0 );
		void moveToWithYaw( const Eigen::Vector3d dest, const double yaw , const double vel = 0.0 );
		void moveTo( const Eigen::Vector3d dest, const double vel = 0.0 );

		//Rotate with a desired yaw and velocity
		void rotate( const double angle, double vel = 0.0); 

		//Land at a desired altitude with desired velocity
		void land( double altitude = 0.0, const double vel = 0.0 );

		//A test routine
		void select_action();

		//Activate or disable trajectory generator
		void activateTrajectoryGenerator( const bool act) { _act_traj_gen = act;}

		//Get functions
		const bool getTakeoff() {return _take_off;}
		const Eigen::Vector4d getWorldPos() { return _world_pos; }
		const double getYaw() { return _mes_yaw; }
		const Eigen::Matrix4d getWorldOffset() { return _world_offset; }

		//Set functions
		void setWorldOffset(const Eigen::Ref<Eigen::Matrix<double, 4, 4>> offset);
		//Set test mode to 'true' to start with state machine
		void setTestMode( bool mode) { _test_mode = mode; }

		void setPointPublisher();
	
	private:
		void pose_cb ( geometry_msgs::PoseStamped msg );
		void mavros_state_cb( mavros_msgs::State mstate);
		void trajectoryGenerator(const Eigen::Vector3d pos, const double yaw, const double vel);
		void trajectoryGeneratorWps(const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> pos, const Eigen::Ref<Eigen::VectorXd> yaw , const double vel);


		ros::NodeHandle _nh;
		ros::Publisher _setpoint_pub;
		ros::Subscriber _pose_sub;
		ros::Subscriber _mavros_state_sub;
      bool _first_local_pos;
		bool _test_mode;

		string _setpoint_topic;
		string _mavros_state_topic;
		string _pose_topic;

		// --- Desired state
		Vector3d _pos_sp;
		double _yaw_sp;
		double _sp_rate;
		
		// --- Drone state ---
		Vector4d _world_pos;
		Matrix4d _world_offset;
		Vector4d _world_quat;
		float _mes_yaw;
      mavros_msgs::State _mstate;
      bool _take_off;
		bool _act_traj_gen;

		// --- Clients services
		ros::ServiceClient _arming_client;
		ros::ServiceClient _set_mode_client;
		ros::ServiceClient _land_client;

      // --- Trajectory planner ---
      CARTESIAN_PLANNER *_trajectory = NULL;
		Eigen::Vector3d _setpoint;
		double _cruise_vel;
		double _traj_rate;

		// --- Thresholds ---
		double _pos_threshold;
		double _yaw_threshold;
		double _height_threshold;
		double _max_height;

};