#include "ros/ros.h"
#include "aruco_msgs/MarkerArray.h"
#include "leonardo_2/navigation.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "boost/thread.hpp"

using namespace std;

class ArucoManager{
	public:
		ArucoManager();

		bool getActualMarkers(std::vector<aruco_msgs::Marker> markers);
		bool landOnMarker(const int id);
		bool correctWorldTransform(bool hard, bool correct_z );
		bool isKnown(int ID);
		bool isSeen(int ID);
		void Routine();
		bool getKnownMarkerPos( const int id, Eigen::Vector3d& pos);
		bool moveToMarker(const int id, const double height);

		bool getNearestMarker( aruco_msgs::Marker& marker );
		bool getNearestKnownMarker( aruco_msgs::Marker& marker );
		bool getMarker( const int id, aruco_msgs::Marker& marker);

		void localize(bool hard, bool correct_z);

		const Eigen::MatrixXd load_wps(int i1, int i2);
		const Eigen::MatrixXd load_research(int i);
		
		bool setDesiredHeight(const double h) { _des_height = h; }
		
		void run();

	private:
		void load_list();
		void markers_cb(aruco_msgs::MarkerArray markers);
		void seq_cb(std_msgs::Float32MultiArray seq);
		void visualServoing();
		void worldTransformFilter();

		ros::NodeHandle _nh;
		ros::Subscriber _markers_sub;
		ros::Subscriber _markers_list_sub;
		ros::Subscriber _seq_sub;
		ros::Publisher _ocr_pub;

		std::vector<aruco_msgs::Marker> _actual_markers;
		std::vector<aruco_msgs::Marker> _knownList; //List of known markers
		double _des_height;

		string _markers_topic_name;
		bool _no_markers;
		double _marker_time_dt;

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
		int _corrections;
		double _norm_threshold;
		double _cobot_1;
		double _cobot_2;

		double _Kp_vs;
		double _Kd_vs;

		bool _atterraggi;
		bool _command_arrived;
		bool _corr_z;

		std_msgs::Float32MultiArray _sequenza;

};
