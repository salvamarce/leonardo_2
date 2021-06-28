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

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

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

		//Activate or disable trajectory generator
		void activateTrajectoryGenerator( const bool act) { _act_traj_gen = act;}

		//Get functions
		const bool getTakeoff() {return _take_off;}
		const Eigen::Vector4d getWorldPos() { return _world_pos; }
		const Eigen::Vector4d getWorldPosOdom() { return _world_pos_odom; }
		const double getYaw() { return _mes_yaw; }
		const double getYawOdom() { return _mes_yaw_odom; }
		const Eigen::Vector4d getWorldQuatOdom() { return _world_quat_odom; }
		const Eigen::Vector4d getWorldQuat() { return _world_quat; }
		const Eigen::Matrix4d getWorldTransform() { return _H_odom_arena; }

		//Set functions
		void setWorldTransform(const Eigen::Ref<Eigen::Matrix<double, 4, 4>> new_H_odom_Arena);

		void setPointPublisher();

		void tf_broadcast_pose_arena();
		void tf_broadcast_pose_odom();
		void tf_broadcast_odom_arena();
		void rviz_sp_publisher();
		void tf_broadcast_poses();
	private:
		void pose_cb ( geometry_msgs::PoseStamped msg );
		void mavros_state_cb( mavros_msgs::State mstate);
		void trajectoryGenerator(const Eigen::Vector3d pos, const double yaw, const double vel);
		void trajectoryGeneratorWps(const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> pos, const Eigen::Ref<Eigen::VectorXd> yaw , const double vel);


		ros::NodeHandle _nh;
		ros::Publisher _setpoint_pub;
		ros::Subscriber _pose_sub;
		ros::Subscriber _mavros_state_sub;
		tf::TransformBroadcaster _broadcaster;
      bool _first_local_pos;

		string _setpoint_topic;
		string _mavros_state_topic;
		string _pose_topic;

		// --- Desired state
		Vector3d _pos_sp;
		double _yaw_sp;
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
		double _drone_height;

};
