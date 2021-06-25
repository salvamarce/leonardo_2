#include "aruco_manager.h"

void load_param( double & p, double def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}
void load_param( string & p, string def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}
void load_param( int & p, int def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}
void load_param( bool & p, bool def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

ArucoManager::ArucoManager( bool test_mode){
   _nh.getParam("markers_topic_name", _markers_topic_name);
	_nh.getParam("markers_list_topic_name", _markers_list_topic_name);
	_nh.getParam("rate_servoing", _rate_servoing);
	_nh.getParam("theta_max", _theta_max);
	_nh.getParam("vel_max", _vel_max);
	_nh.getParam("min_height", _min_height);
	_nh.getParam("Kp_vs", _Kp_vs);
	_nh.getParam("Kd_vs", _Kd_vs);
	//load_wps();
	load_list(); 

	_markers_sub = _nh.subscribe(_markers_topic_name.c_str(), 1, &ArucoManager::markers_cb, this);
	_markers_list_sub = _nh.subscribe(_markers_list_topic_name.c_str(), 1, &ArucoManager::markers_list_cb, this);

	nvg = new Navigation;
	nvg->setTestMode(false);
	_visual_servoing = false;
	_land_on_marker = false;
	_no_markers = true;
	_des_height =  0.0;

	_test_mode = test_mode;
}

void ArucoManager::load_wps(){

	std::vector<double> wps;
	wps.resize(30*3);
	_wps_1to10.setZero(3,wps.size()/3);

	if (_nh.hasParam("wps_1to10")){

		_nh.getParam("wps_1to10", wps);

		int cols = _wps_1to10.cols();
		
		for (int i = 0; i < (30*3 - 2); i+=3)
		{
			_wps_1to10(0, i/3) = wps[i];
			_wps_1to10(1, i/3) = wps[i+1];
			_wps_1to10(2, i/3) = wps[i+2];
			
		}
		
	 }

}

void ArucoManager::load_list(){

	_knownList.resize(10);
	string pos;
	int id;
	double x, y, z;

	for(int i=0; i<10; i++){
		pos = std::to_string(i+1);

		_nh.getParam("marker_"+pos+"_id", id);
		_knownList[i].id = id;

		_nh.getParam("marker_"+pos+"_x", x);
		_knownList[i].pose.pose.position.x = x;

		_nh.getParam("marker_"+pos+"_y", y);
		_knownList[i].pose.pose.position.y = y;

		_nh.getParam("marker_"+pos+"_z", z);
		_knownList[i].pose.pose.position.z = z;

		cout << i << ": " << _knownList[i].id << "-> " << _knownList[i].pose.pose.position.x << " "
		 << _knownList[i].pose.pose.position.y << " " << _knownList[i].pose.pose.position.z << endl; 
	}
	
}

void ArucoManager::markers_cb(aruco_msgs::MarkerArray markers){
	bool known = false;
	bool seen = false;

	if( markers.markers.size() > 0 ){
		for(int i=0; i < markers.markers.size(); i++){
			known = false;
			seen = false;

			if(isKnown(markers.markers[i].id)){
				known = true;
				_actual_marker = markers.markers[i];
				
				//if( nvg->getTakeoff() )
				//	correctDronePosition();

			}

			if(!known && markers.markers[i].id != 0){} //To do: fai qualcosa con l'intruso
				//cout << "Marker " << markers.markers[i].id << " unknown, INTRUDER!! \n";
		}
	}
}

void ArucoManager::markers_list_cb(std_msgs::UInt32MultiArray list){
	if(list.data.size() > 0)
		_no_markers = false;
	else
		_no_markers = true;
}

bool ArucoManager::isKnown(int ID){
	bool found = false;
	int i=0;

	while(!found && i<_knownList.size()){
		if(ID == _knownList[i].id)
			found = true;
		
		i++;
	}

	return found;
}

bool ArucoManager::isSeen(int ID){
	bool seen = false;
	int i=0;

	while(!seen && i<_seenList.size()){
		if(ID == _seenList[i].id)
			seen = true;
		
		i++;
	}

	return seen;
}

bool ArucoManager::getActualMarker(aruco_msgs::Marker& marker){
	if( !_no_markers ){
		marker = _actual_marker;
		return true;
	}
	else
		return false;
}

bool ArucoManager::getKnownMarkerPos( const int id, Eigen::Vector3d& pos){

	for(int i=0; i<_knownList.size(); i++){
		if(id == _knownList[i].id){
			pos(0) = _knownList[i].pose.pose.position.x;
			pos(1) = _knownList[i].pose.pose.position.y;
			pos(2) = _knownList[i].pose.pose.position.z;

			return true;
		}
	}

	return false;
}

bool ArucoManager::correctDronePosition(){
	aruco_msgs::Marker marker;
	Eigen::Vector3d dist;
	Eigen::Vector3d marker_pos;
	Eigen::Vector4d marker_dir;
	Eigen::Matrix3d R_marker;
	Eigen::Matrix3d R_marker_D;
	Eigen::Matrix4d H_marker_A;
	Eigen::Matrix4d H_marker_D;
	Eigen::Matrix4d H_D_A;
	

	R_marker << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	
	if( getActualMarker(marker) ){
		
		if( isKnown(marker.id) ){
			
			getKnownMarkerPos(marker.id, marker_pos);
			dist << marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z;
			marker_dir << marker.pose.pose.orientation.w, marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z;			
			
			H_marker_A.block<3,3>(0,0) = R_marker;
			H_marker_A.block<3,1>(0,3) = marker_pos;
			H_marker_A(3,0) = 0.0;
			H_marker_A(3,1) = 0.0;
			H_marker_A(3,2) = 0.0;
			H_marker_A(3,3) = 1.0; 

			R_marker_D = utilities::QuatToMat( marker_dir );

			H_marker_D.block<3,3>(0,0) = R_marker_D;
			H_marker_D.block<3,1>(0,3) = dist;
			H_marker_D(3,0) = 0.0;
			H_marker_D(3,1) = 0.0;
			H_marker_D(3,2) = 0.0;
			H_marker_D(3,3) = 1.0;

			H_D_A = H_marker_A * H_marker_D.inverse();
			//H_D_A(2,3) = 0.0;

			cout << "marker_pos: " << marker_pos.transpose() << " norm: " << marker_pos.norm() << endl;
			cout << "dist: " << dist.transpose() << " norm: " << dist.norm() << endl;
			cout << "odom_pos: " << nvg->getWorldPosOdom().transpose() << " norm: " << nvg->getWorldPosOdom().norm() << endl;
			cout << "H_D_A: \n" << H_D_A << endl;

			nvg->setWorldOffset( H_D_A );

			return true;
			
		}
		else
			return false;
	}
	else
		return false;
}

bool ArucoManager::moveToMarker(const int id, const double height){
	aruco_msgs::Marker marker;
	Eigen::Vector3d target;

	if( nvg->getTakeoff() ){

		if( getActualMarker(marker) && marker.id == id ){
			cout << "sono sul marker richiesto \n";
			return true;
		}
		else if( getKnownMarkerPos(id, target)){
			cout << "Vado al marker richiesto \n";
			target(2) = height;
			cout << "marker target: " << target.transpose() << endl;
			nvg->moveTo(target);

			return true;
		}
		else
			return false;
	}
}

void ArucoManager::visualServoing(){
	
	aruco_msgs::Marker marker;
	Eigen::Vector3d dist_marker;
	Eigen::Vector3d old_dist;
	Eigen::Vector3d target;

	Eigen::Matrix4d world_offset;
	Eigen::Vector4d world_pos;

	Eigen::Vector3d des_vec;
	double cos_theta;
	double theta;
	double vel;
	double cos_yaw, sin_yaw, drone_yaw;
	Eigen::Matrix3d Rdrone;
	Eigen::Matrix3d Rmarker;

	double yaw = 0.0;

	ros::Rate r(_rate_servoing);
	double dt = 1.0/_rate_servoing;

	old_dist << 0.0, 0.0, 0.0;

	Rmarker << 1, 0, 0, 0, cos(3.14), -1.0*sin(3.14), 0, sin(3.14), cos(3.14);

	while(ros::ok()){
		
		if( _visual_servoing ){
			
			if( getActualMarker(marker) && marker.id == _servoing_marker_id ){
				
				// Tutto calcolato in terna odom
				world_pos = nvg->getWorldPos();
				world_offset = nvg->getWorldOffset();
				
				//Get distance from marker
				dist_marker << marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z;

				drone_yaw = nvg->getYaw();
				cos_yaw = cos(drone_yaw);
				sin_yaw = sin(drone_yaw);
				Rdrone << cos_yaw, -1.0*sin_yaw, 0, sin_yaw, cos_yaw, 0, 0, 0, 1;

				dist_marker = Rmarker * Rdrone * dist_marker;

				if(_land_on_marker){
					//Evaluate landing speed depending on the position of the drone inside the cone			
					des_vec << 0.0, 0.0, dist_marker(2);
					cos_theta = des_vec.dot(dist_marker) / (des_vec.norm() * dist_marker.norm());
					theta = acos(cos_theta);
					vel = (-_vel_max/_theta_max) * theta + _vel_max;
				}

				//TO DO: implementare un PD per il visual servoing
				cout << "dist marker: " << dist_marker.transpose() << endl;
				cout << "theta: " << theta << endl;
				if( !_land_on_marker ){
					//Turn off trajectory planner
					nvg->activateTrajectoryGenerator(false);

					//X-Y visual servoing
					target = ( world_offset.transpose() * world_pos ).block<3,1>(0,0)  + _Kp_vs * dist_marker + _Kd_vs * ( ( dist_marker - old_dist ) / dt ) ;
					target(2) = _des_height - world_offset(2,3) ;
					nvg->moveToWithYaw(target, yaw);
					old_dist = dist_marker;

				}
				else if( _land_on_marker && ( (dist_marker(2) > _min_height) && (theta < _theta_max) ) ){

					//Turn off trajectory planner
					nvg->activateTrajectoryGenerator(false);	

					//X-Y visual servoing and descent
					target = ( world_offset.transpose() * world_pos ).block<3,1>(0,0)  + _Kp_vs * dist_marker + _Kd_vs * ( ( dist_marker - old_dist ) / dt ) ;
					target(2) = dist_marker(2) + vel * dt;
					cout << "dist: " << dist_marker(2) << endl;
					cout << "vel: " << vel << endl;
					cout << "dz: " << vel*dt << endl;
					nvg->moveToWithYaw(target, yaw);
					old_dist = dist_marker;

				}
				else if( _land_on_marker && ( (dist_marker(2) > _min_height) && (theta > _theta_max) ) ){

					_visual_servoing = false;
					nvg->activateTrajectoryGenerator(true);
					moveToMarker(_servoing_marker_id, 2.0);
					landOnMarker(_servoing_marker_id);

				}
				else if( _land_on_marker && ( (dist_marker(2) <= _min_height) && (theta < _theta_max) ) ){

					//Stop visual servoing and just land
					_visual_servoing = false;
					nvg->activateTrajectoryGenerator(true);
					double height_land = world_pos(2) - world_offset(2,3) - dist_marker(2);
					nvg->land( height_land , 0.3 );
				}

			}
			else if( _no_markers && nvg->getWorldPos()(2) > _min_height ){

				nvg->activateTrajectoryGenerator(true);
				_visual_servoing = false;
				moveToMarker(_servoing_marker_id, 2.0);
				landOnMarker(_servoing_marker_id);

			}
			else if( _no_markers && nvg->getWorldPos()(2) <= _min_height ){
				
				//Stop visual servoing and just land
				_visual_servoing = false;
				nvg->activateTrajectoryGenerator(true);
				double height_land = world_pos(2) - world_offset(2,3) - dist_marker(2);
				nvg->land( height_land , 0.3 );
			} 		
		}
		else{
			nvg->activateTrajectoryGenerator(true);
		}
		
		r.sleep();
	}

}

bool ArucoManager::landOnMarker(const int id){
	aruco_msgs::Marker marker;
	Eigen::Vector3d target;

	if( nvg->getTakeoff() ){

		if( getActualMarker(marker) && marker.id == id ){
			cout << "sto già sul marker \n";
			_servoing_marker_id = id;
			_land_on_marker = true;
			_visual_servoing = true;
			return true;

		}
		else if( getKnownMarkerPos(id, target) && nvg->getWorldPos()(2) > _min_height){
			cout << "vado al marker \n";
			while(!moveToMarker(id, 2.0)) sleep(0.01);
			_servoing_marker_id = id;
			_land_on_marker = true;
			_visual_servoing = true;
			return true;

		}
		else
			return false;
	}
}

void ArucoManager::Routine(){
	aruco_msgs::Marker mark;
	Eigen::Vector3d pos;
	bool landed = false;
	bool seq_finished = false;
	bool moved = false;
	bool first_takeoff = false;

	ros::Rate r(10);

	_des_height = 3.5;

	while( ros::ok() ){

		if( !first_takeoff ){
			cout << "TAKEOFF \n";
			nvg->takeoff(_des_height, 0.4);

			if( correctDronePosition() )
				first_takeoff = true;
			else
				nvg->land();

			sleep(1.0);
		}
		else{

			if(!seq_finished){
				cout << "Sequenza \n";
				//pos << 2.0, 2.0, 3.5;
				//nvg->moveTo(pos);

				//nvg->moveToWithYaw(pos, 1.57);
				//sleep(1.0);
				
				moveToMarker(2, 3.5);
				
				sleep(2.0);

				
				correctDronePosition();
				
				sleep(2.0);

				moveToMarker(1, 3.5);
				
				sleep(2.0);
				
				seq_finished = true;
				cout << "Sequenza terminata \n";
				
			}
			
			if(seq_finished){
				nvg->land();
			}
			
		}

		r.sleep();
	}
}

void ArucoManager::TestRoutine(){

	ros::Rate r(1);

	bool sp = false;
	Eigen::Vector4d pos;
	string key;

	pos << 1.0, 2.0, 2.0, 1.0;

	while(ros::ok()){
		
		getline(cin, key);

		if(key == "p"){
			correctDronePosition();

			cout << "world pos: " << nvg->getWorldPos().transpose() << endl;
			cout << "odom_pos: " << (nvg->getWorldOffset().inverse() * nvg->getWorldPos()).transpose() << endl;
			cout << "yaw: " << nvg->getYaw() << endl;
			cout << "sp: " << (nvg->getWorldOffset().inverse() * pos).transpose() << endl;
			cout << " \n ---- \n";
		}


		//r.sleep();
	}

}


 /*
void double_camera_manager::tf_broadcast_pose(geometry_msgs::PoseStamped p, std::string child_frame, std::string parent_frame){

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(p.pose.position.x,p.pose.position.y,p.pose.position.z));
    tf::Quaternion q(p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w);
    transform.setRotation(q);
    tf::StampedTransform stamp_transform(transform, ros::Time::now(), parent_frame, child_frame);
    _broadcaster.sendTransform(stamp_transform);

}
*/
void ArucoManager::run(){

	cout << "test_mode: " << _test_mode << endl;

	if( !_test_mode){

		boost::thread traj_gen_t( &Navigation::setPointPublisher, nvg);
		sleep(1);
		//boost::thread visual_t( &ArucoManager::visualServoing, this);
		boost::thread routine_t( &ArucoManager::Routine, this);
		
	}
	else if( _test_mode ){
		boost::thread test_routine_t( &ArucoManager::TestRoutine, this);
	}
	ros::spin();
}


