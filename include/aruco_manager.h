#include "ros/ros.h"
#include "aruco_msgs/MarkerArray.h"
#include "navigation.h"
#include "std_msgs/UInt32MultiArray.h"
#include "boost/thread.hpp"

using namespace std;

class ArucoManager{
	public:
		ArucoManager();

		bool getActualMarkerPos(const aruco_msgs::Marker &marker, Eigen::Vector3d &pos);
		bool getActualMarker(aruco_msgs::Marker &marker);
		bool landOnMarker(const int id);
		void correctDronePosition();
		bool isKnown(int ID);
		bool isSeen(int ID);
		void TestRoutine();
		bool getKnownMarkerPos( const int id, Eigen::Vector3d &pos );
		bool moveToMarker(const int id);
		
		bool setDesiredHeight(const double h) { _des_height = h; }
		
		void run();

	private:
		void load_list();
		void markers_cb(aruco_msgs::MarkerArray markers);
		void markers_list_cb(std_msgs::UInt32MultiArray list);
		void visualServoing();

		std::vector<aruco_msgs::Marker> _knownList; //List of known markers
		std::vector<aruco_msgs::Marker> _seenList;
		ros::NodeHandle _nh;
		ros::Subscriber _markers_sub;
		ros::Subscriber _markers_list_sub;

		aruco_msgs::Marker _actual_marker;
		double _des_height;

		string _markers_topic_name;
		string _markers_list_topic_name;
		bool _no_markers;

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