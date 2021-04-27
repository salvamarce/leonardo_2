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

ArucoManager::ArucoManager(){
   _nh.getParam("markers_topic_name", _markers_topic_name);
	_nh.getParam("markers_list_topic_name", _markers_list_topic_name);
	_nh.getParam("rate_servoing", _rate_servoing);
	_nh.getParam("theta_max", _theta_max);
	_nh.getParam("vel_max", _vel_max);
	_nh.getParam("min_height", _min_height);
	_nh.getParam("Kp_vs", _Kp_vs);
	_nh.getParam("Kd_vs", _Kd_vs);
	load_list(); 

	_markers_sub = _nh.subscribe(_markers_topic_name.c_str(), 1, &ArucoManager::markers_cb, this);
	_markers_list_sub = _nh.subscribe(_markers_list_topic_name.c_str(), 1, &ArucoManager::markers_list_cb, this);

	nvg = new Navigation;
	nvg->setTestMode(false);
	_visual_servoing = false;
	_land_on_marker = false;
	_no_markers = true;
	_des_height =  0.0;
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
		for(int i=0; i<markers.markers.size(); i++){
			known = false;
			seen = false;

			if(isKnown(markers.markers[i].id)){
				known = true;
				_actual_marker = markers.markers[i];
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

bool ArucoManager::getActualMarkerPos(const aruco_msgs::Marker &marker, Eigen::Vector3d &pos){
	if( !_no_markers ){
		//Marker is in ENU
		//
		//marker = _actual_marker;
		return true;
	}
	else
		return false;
			
}

bool ArucoManager::getActualMarker(aruco_msgs::Marker &marker){
	if( !_no_markers ){
		marker = _actual_marker;
		return true;
	}
	else
		return false;
}

bool ArucoManager::getKnownMarkerPos( const int id, Eigen::Vector3d &pos){

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

void ArucoManager::correctDronePosition(){
	aruco_msgs::Marker mark;

	if( getActualMarker(mark) ){
		
		if( isKnown(mark.id) ){

			for(int i=0; i<_knownList.size(); i++){
				if(mark.id == _knownList[i].id){
					Eigen::Vector3d pos;
					pos << _knownList[i].pose.pose.position.x + mark.pose.pose.position.x, _knownList[i].pose.pose.position.y + mark.pose.pose.position.y, 0.0;
					nvg->setWorldOffset( pos );
				}
			}
		}
	}
}

bool ArucoManager::moveToMarker(const int id){
	aruco_msgs::Marker marker;
	Eigen::Vector3d target;

	if( nvg->getTakeoff() ){

		if( getActualMarker(marker) && marker.id == id ){
			cout << "sono sul marker richiesto \n";
			return true;
		}
		else if( getKnownMarkerPos(id, target)){
			cout << "Vado al marker richiesto \n";
			target(2) = _des_height;
			nvg->move_to(target);
			while(!nvg->arrived()) usleep(1*1e04);
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

	Eigen::Vector3d world_offset;
	Eigen::Vector3d world_pos;

	Eigen::Vector3d des_vec;
	double cos_theta;
	double theta;
	double vel;

	ros::Rate r(_rate_servoing);
	double dt = 1.0/_rate_servoing;

	old_dist << 0.0, 0.0, 0.0;

	while(ros::ok()){
		
		if( _visual_servoing ){
			
			if( getActualMarker(marker) && marker.id == _servoing_marker_id ){
				// Tutto calcolato in terna odom
				world_pos = nvg->getWorldPos();
				world_offset = nvg->getWorldOffset();
				
				//Get distance from marker
				dist_marker << marker.pose.pose.position.x, marker.pose.pose.position.y, -1.0*marker.pose.pose.position.z;
				//marker_quat << marker.pose.pose.orientation.w, marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z;
				
				if(_land_on_marker){

					//Evaluate landing speed depending on the position of the drone inside the cone			
					des_vec << 0.0, 0.0, dist_marker(2);
					cos_theta = des_vec.dot(dist_marker) / (des_vec.norm() * dist_marker.norm());
					theta = acos(cos_theta);
					vel = (-_vel_max/_theta_max) * theta + _vel_max;

				}

				//TO DO: implementare un PD per il visual servoing
				
				if( !_land_on_marker ){
					//Turn off trajectory planner
					nvg->activateTrajectoryGenerator(false);

					//X-Y visual servoing
					target = (world_pos - world_offset)  + _Kp_vs * dist_marker + _Kd_vs * ( ( dist_marker - old_dist ) / dt ) ;
					target(2) = _des_height - world_offset(2) ;
					nvg->move_to(target);
					old_dist = dist_marker;

				}
				else if( _land_on_marker && ( (dist_marker(2) > _min_height) && (theta < _theta_max) ) ){

					//Turn off trajectory planner
					nvg->activateTrajectoryGenerator(false);	

					//X-Y visual servoing and descent
					target = (world_pos - world_offset)  + _Kp_vs * dist_marker + _Kd_vs * ( ( dist_marker - old_dist ) / dt ) ;
					target(2) = dist_marker(2) - vel * dt;
					cout << "dist: " << dist_marker(2) << endl;
					cout << "vel: " << vel << endl;
					cout << "dz: " << -vel*dt << endl;
					nvg->move_to(target);
					old_dist = dist_marker;

				}
				else if( _land_on_marker && ( (dist_marker(2) > _min_height) && (theta > _theta_max) ) ){

					_visual_servoing = false;
					nvg->activateTrajectoryGenerator(true);
					moveToMarker(_servoing_marker_id);
					landOnMarker(_servoing_marker_id);

				}
				else if( _land_on_marker && ( (dist_marker(2) <= _min_height) && (theta < _theta_max) ) ){

					//Stop visual servoing and just land
					_visual_servoing = false;
					nvg->activateTrajectoryGenerator(true);
					double height_land = world_pos(2) - world_offset(2) - dist_marker(2);
					nvg->land( height_land , 0.3 );
				}

			}
			else if( _no_markers && nvg->getWorldPos()(2) > _min_height ){

				nvg->activateTrajectoryGenerator(true);
				_visual_servoing = false;
				moveToMarker(_servoing_marker_id);
				landOnMarker(_servoing_marker_id);

			}
			else if( _no_markers && nvg->getWorldPos()(2) <= _min_height ){
				
				//Stop visual servoing and just land
				_visual_servoing = false;
				nvg->activateTrajectoryGenerator(true);
				double height_land = world_pos(2) - world_offset(2) - dist_marker(2);
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

			_servoing_marker_id = id;
			_visual_servoing = true;
			_land_on_marker = true;
			return true;

		}
		else if( getKnownMarkerPos(id, target) && nvg->getWorldPos()(2) > _min_height){

			while(!moveToMarker(id)) sleep(0.01);
			_servoing_marker_id = id;
			_visual_servoing = true;
			_land_on_marker = true;
			return true;

		}
		else
			return false;
	}
}

void ArucoManager::TestRoutine(){
	aruco_msgs::Marker mark;
	bool corrected = false;
	bool landed = false;
	bool moved = false;

	ros::Rate r(50);

	_des_height = 2.0;

	while( ros::ok() ){
		if( !nvg->getTakeoff() ){
			nvg->takeoff(_des_height);
			sleep(0.5);
		}
		else{
			
			if(!corrected){
				/**
				 * TO DO:
				 * creare un modulo che continuamente corregge
				 * la posizione del drone
				 **/ 
				correctDronePosition();
				sleep(0.5);
				corrected = true;

			}

			for(int i = 1; i<= 10; i++){
				while( !moveToMarker(i) )
					sleep(0.01);				
			}
			
			/*	
			if(!landed){
				if( landOnMarker(1) ){
					landed = true;
				}
			}
			
			*/
		}

		r.sleep();
	}
}

void ArucoManager::run(){
	boost::thread traj_gen_t( &Navigation::trajectoryGenerator, nvg);
	sleep(1);
	boost::thread visual_t( &ArucoManager::visualServoing, this);
	boost::thread routine_t( &ArucoManager::TestRoutine, this);
	ros::spin();
}


