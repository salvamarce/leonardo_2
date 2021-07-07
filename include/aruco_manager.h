#include "ros/ros.h"
#include "aruco_msgs/MarkerArray.h"
#include "navigation.h"
#include "std_msgs/UInt32MultiArray.h"
#include "boost/thread.hpp"

using namespace std;

class ArucoManager{
	public:
		ArucoManager( bool test_mode = false );

		bool getActualMarkers(std::vector<aruco_msgs::Marker> markers);
		bool landOnMarker(const int id);
		bool correctWorldTransform(bool hard = false );
		bool isKnown(int ID);
		bool isSeen(int ID);
		void TestRoutine();
		void Routine();
		bool getKnownMarkerPos( const int id, Eigen::Vector3d& pos);
		bool moveToMarker(const int id, const double height);

		bool getNearestMarker( aruco_msgs::Marker& marker );
		bool getNearestKnownMarker( aruco_msgs::Marker& marker );
		bool getMarker( const int id, aruco_msgs::Marker& marker);

		
		bool setDesiredHeight(const double h) { _des_height = h; }
		
		void run();

	private:
		void load_list();
		void load_wps();
		void markers_cb(aruco_msgs::MarkerArray markers);
		void visualServoing();
		//void worldTransformFilter();

		bool _test_mode;

		ros::NodeHandle _nh;
		ros::Subscriber _markers_sub;
		ros::Subscriber _markers_list_sub;

		std::vector<aruco_msgs::Marker> _actual_markers;
		std::vector<aruco_msgs::Marker> _knownList; //List of known markers
		double _des_height;

		string _markers_topic_name;
		bool _no_markers;

		Eigen::Matrix<double, 3, Dynamic> _wps_9to3;
		Eigen::Matrix<double, 3, Dynamic> _wps_3to2;
		Eigen::Matrix<double, 3, Dynamic> _wps_2to6;

		Eigen::Matrix4d _H_odom_arena_sp;
		double _transform_filter_rate;
		bool _continous_correction;
		bool _new_transform;

		// --- Visual Servoing ---
		Navigation *nvg = NULL;
		bool _visual_servoing;
		bool _land_on_marker;
		double _rate_servoing;
		double _theta_max;
		double _vel_max;
		double _min_height;
		int _servoing_marker_id;

		double _Kp_vs;
		double _Kd_vs;

};
