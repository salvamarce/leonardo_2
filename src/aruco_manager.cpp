#include "aruco_manager.h"

ArucoManager::ArucoManager(){
   _nh.getParam("markers_topic_name", _markers_topic_name);
	_nh.getParam("rate_servoing", _rate_servoing);
	_nh.getParam("theta_max", _theta_max);
	_nh.getParam("vel_max", _vel_max);
	_nh.getParam("min_height", _min_height);
	_nh.getParam("Kp_vs", _Kp_vs);
	_nh.getParam("Kd_vs", _Kd_vs);
	_nh.getParam("norm_threshold", _norm_threshold);
	_nh.getParam("cobot_1", _cobot_1);
	_nh.getParam("cobot_2", _cobot_2);
	_nh.getParam("transform_filter_rate", _transform_filter_rate);
	_nh.getParam("marker_time_dt", _marker_time_dt);
	_nh.getParam("corr_z", _corr_z);
	load_list(); 

	_markers_sub = _nh.subscribe(_markers_topic_name.c_str(), 1, &ArucoManager::markers_cb, this);
	_seq_sub = _nh.subscribe("/leonardo/landing_sequence", 1, &ArucoManager::seq_cb, this);
	_ocr_pub = _nh.advertise<std_msgs::Bool>( "/ocr/active" , 1);

	nvg = new Navigation;
	_visual_servoing = false;
	_land_on_marker = false;
	_no_markers = true;
	_des_height =  0.0;

	_continous_correction = true;
	_new_transform = false;

	_H_odom_arena_sp = Eigen::Matrix4d::Identity();
	
	_actual_markers.clear();
	_actual_markers.resize(0);

	_corrections = 0;

	_atterraggi = false;
	_command_arrived = false;
	
}

const Eigen::MatrixXd ArucoManager::load_wps(int i1, int i2){

	std::vector<double> wps;
	Eigen::MatrixXd _wps_;
	string name;

	name = "wps_"+std::to_string(i1)+"to"+std::to_string(i2);

	if (_nh.hasParam(name.c_str())){

		_nh.getParam(name.c_str(), wps);

		int col = wps.size()/3;

		_wps_.resize(3,col);
		
		for (int i = 0; i < (3*col - 2); i+=3)
		{
			_wps_(0, i/3) = wps[i];
			_wps_(1, i/3) = wps[i+1];
			_wps_(2, i/3) = wps[i+2];
			
		}
		
	 }
	 cout << name << ": \n" << _wps_ << endl;

	 return _wps_;

}

const Eigen::MatrixXd ArucoManager::load_research(int i){

	std::vector<double> wps;
	Eigen::MatrixXd _wps_;
	string name;

	name = "research_path_"+std::to_string(i);

	if (_nh.hasParam(name.c_str())){

		_nh.getParam(name.c_str(), wps);

		int col = wps.size()/3;

		_wps_.resize(3,col);
		
		for (int i = 0; i < (3*col - 2); i+=3)
		{
			_wps_(0, i/3) = wps[i];
			_wps_(1, i/3) = wps[i+1];
			_wps_(2, i/3) = wps[i+2];
			
		}
		
	 }
	 cout << name << ": \n" << _wps_ << endl;

	 return _wps_;

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
	
	if( !nvg->getTakeoff() ) return;

	bool seen = false;

	_actual_markers.clear();
	_actual_markers.resize( markers.markers.size() );

	if( markers.markers.size() > 0 ){

		for(int i=0; i < markers.markers.size(); i++){

			seen = false;

			if( markers.markers[i].id == 97 && markers.markers[i].id == 0) return; /* read errors */

			_actual_markers.push_back( markers.markers[i] );

			if(isKnown(markers.markers[i].id)){
					
				// if ( nvg->getTakeoff() && _continous_correction && !_visual_servoing){
				// 	correctWorldTransform(false,false);
				// }

			}
			else{

				if( !_visual_servoing && !nvg->getInterrupt() && !_atterraggi &&
					 markers.markers[i].id != _cobot_1 && markers.markers[i].id != _cobot_2 && 
					 (markers.markers[i].id >=21 && markers.markers[i].id <= 26 || markers.markers[i].id == 177) ){

					aruco_msgs::Marker intr_mark;
					intr_mark = markers.markers[i];
					cout << "interrompo, marker id: " << intr_mark.id << endl;
					
					if( !nvg->getInterrupt() && !_visual_servoing ){
						
						nvg->interruptAll();
						sleep(0.5);
						Eigen::Vector3d pos;
						pos << intr_mark.pose.pose.position.x, intr_mark.pose.pose.position.y, intr_mark.pose.pose.position.z;
						cout << "posizione_intruso: " << pos.transpose() << endl;

						// nvg->moveTo(pos);

						std_msgs::Bool ocr;
						ocr.data = true;
						_ocr_pub.publish(ocr);
						// nvg->interruptAll();
						sleep(0.2);

						_land_on_marker = false;
						_servoing_marker_id = 177;
						// nvg->resetInterrupt();
						_visual_servoing = true;
					}
					
				}

			} 
		}

	}
}

void ArucoManager::seq_cb(std_msgs::Float32MultiArray seq){

	if( seq.data.size() > 0){
		
		if( seq.data.size() == 1)
			_command_arrived = true;
		else{
			// nvg->interruptAll();
			_sequenza = seq;
			_command_arrived = true;
			_atterraggi = true;
		}
		sleep(0.5);
		nvg->resetInterrupt();
	}

}

bool ArucoManager::isKnown(int ID){
	bool found = false;
	int i=0;

	while(!found && i < _knownList.size()){
		
		if(ID == _knownList[i].id )
			found = true;
		
		i++;
	}

	return found;
}

bool ArucoManager::isSeen(int ID){
	bool seen = false;
	int i=0;
	double time, dt;

	time = ros::Time::now().toSec();

	while(!seen && i<_actual_markers.size()){

		dt = time - _actual_markers[i].header.stamp.toSec();

		if(ID == _actual_markers[i].id && dt < _marker_time_dt)
			seen = true;
		
		i++;
	}

	return seen;
}

bool ArucoManager::getActualMarkers( std::vector<aruco_msgs::Marker> markers){

	if( _actual_markers.size() > 0 ){
		
		markers = _actual_markers;
		return true;
	}
	else
		return false;
}

bool ArucoManager::getNearestMarker( aruco_msgs::Marker& marker ){
	int min_norm = 0;
	int norm = 0;
	int nearest_id = 0;
	int nearest_marker_index = -1;
	Eigen::Vector3d dist;
	double time, dt;

	time = ros::Time::now().toSec();

	if( _actual_markers.size() > 0 ){
		
		for( int i=0; i < _actual_markers.size(); i++){

			if( _actual_markers[i].id != 0 ){

				dt = time - _actual_markers[i].header.stamp.toSec(); 

				dist << _actual_markers[i].pose.pose.position.x, _actual_markers[i].pose.pose.position.y, _actual_markers[i].pose.pose.position.z;
				norm = dist.norm();

				if( norm < min_norm && dt < _marker_time_dt ){
					min_norm = norm;
					nearest_id  = _actual_markers[i].id;
					nearest_marker_index = i;
				}
			}
			
		}
		
		if( nearest_marker_index != -1 ){
			marker = _actual_markers[ nearest_marker_index ];
			return true;
		}
		else
			return false;
		
	}
	else
		return false;
}

bool ArucoManager::getNearestKnownMarker( aruco_msgs::Marker& marker ){

	double min_norm = 1000.0;
	double norm = 1000.0;
	int nearest_id = 0;
	int nearest_marker_index = -1;
	Eigen::Vector3d dist;
	double time, dt;

	time = ros::Time::now().toSec();

	if( _actual_markers.size() > 0 ){

		for( int i=0; i < _actual_markers.size(); i++){

			if( isKnown( _actual_markers[i].id ) ){

				dt = time - _actual_markers[i].header.stamp.toSec();
				// cout << "time: " << time << endl;
				// cout << "marker time: " << _actual_markers[i].header.stamp.toSec() << endl;
				// cout << "dt: " << dt << endl;

				if ( dt < _marker_time_dt  ){
					dist << _actual_markers[i].pose.pose.position.x, _actual_markers[i].pose.pose.position.y, _actual_markers[i].pose.pose.position.z;
					norm = dist.norm();

					if( norm < min_norm ){
						min_norm = norm;
						nearest_id  = _actual_markers[i].id;
						nearest_marker_index = i;
						// cout << "marker_id: " << _actual_markers[i].id << endl;
					}
				}
			}
			
		}
		// cout << " nearest_index: " << nearest_marker_index << endl;
		if( min_norm != 1000.0 && nearest_marker_index != -1 ){
			
			marker = _actual_markers[ nearest_marker_index ];

			return true;
		}
		else
			return false;
			
	}
	else
		return false;
}

bool ArucoManager::getKnownMarkerPos( const int id, Eigen::Vector3d& pos){

	for(int i=0; i< _knownList.size(); i++){

		if(id == _knownList[i].id){

			pos(0) = _knownList[i].pose.pose.position.x;
			pos(1) = _knownList[i].pose.pose.position.y;
			pos(2) = _knownList[i].pose.pose.position.z;

			return true;
		}
	}

	return false;
}

bool ArucoManager::getMarker( const int id, aruco_msgs::Marker& marker){

	double time, dt;

	time = ros::Time::now().toSec();

	for(int i=0; i< _actual_markers.size(); i++){
		
		dt = time - _actual_markers[i].header.stamp.toSec();

		if( _actual_markers[i].id == id && dt < _marker_time_dt ){

			marker = _actual_markers[i];

			return true;
		}
	}

	return false;
}

bool ArucoManager::correctWorldTransform( bool hard, bool correct_z){
	
	aruco_msgs::Marker marker;
	Eigen::Vector3d dist;
	Eigen::Vector3d dist_sum;
	Eigen::Vector3d marker_pos;

	Eigen::Vector4d marker_quat;
	Eigen::Vector4d marker_quat_sum;
	Eigen::Vector4d world_pos_odom;

	Eigen::Matrix3d R_marker;
	Eigen::Matrix3d R_marker_drone;
	Eigen::Matrix3d R_drone;

	Eigen::Matrix4d H_marker_arena;
	Eigen::Matrix4d H_marker_drone;
	Eigen::Matrix4d H_drone_arena;
	Eigen::Matrix4d H_drone_odom;
	Eigen::Matrix4d H_odom_arena;	

	//R_marker << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	R_marker = Eigen::Matrix3d::Identity();

	dist_sum = Eigen::Vector3d::Zero();
	marker_quat_sum << 1.0, 0.0, 0.0, 0.0;

	int rate = 15;
	ros::Rate r_sum(rate);
	
	if( getNearestKnownMarker(marker) ){
		
		if( isKnown(marker.id) ){

			world_pos_odom = nvg->getWorldPosOdom();
			getKnownMarkerPos(marker.id, marker_pos);

			for (int i=0; i<rate; i++){

				getNearestKnownMarker(marker);
				dist_sum(0) += marker.pose.pose.position.x;
				dist_sum(1) += marker.pose.pose.position.y;
				dist_sum(2) += marker.pose.pose.position.z;

				marker_quat_sum(0) += marker.pose.pose.orientation.w;
				marker_quat_sum(1) += marker.pose.pose.orientation.x;
				marker_quat_sum(2) += marker.pose.pose.orientation.y;
				marker_quat_sum(3) += marker.pose.pose.orientation.z;		

				r_sum.sleep();	
			}

			dist(0) = dist_sum(0) / rate;
			dist(1) = dist_sum(1) / rate;
			dist(2) = dist_sum(2) / rate;

			marker_quat(0) = marker_quat_sum(0) / rate;
			marker_quat(1) = marker_quat_sum(1) / rate;
			marker_quat(2) = marker_quat_sum(2) / rate;
			marker_quat(3) = marker_quat_sum(3) / rate;
			marker_quat.normalize();

			R_drone = utilities::QuatToMat( nvg->getWorldQuatOdom() );

			H_marker_arena.block<3,3>(0,0) = R_marker;
			H_marker_arena.block<3,1>(0,3) = marker_pos;
			H_marker_arena(3,0) = 0.0;
			H_marker_arena(3,1) = 0.0;
			H_marker_arena(3,2) = 0.0;
			H_marker_arena(3,3) = 1.0; 

			R_marker_drone = utilities::QuatToMat( marker_quat );
			//dist = R_marker_drone.transpose() * dist;

			H_marker_drone.block<3,3>(0,0) = R_marker_drone;
			H_marker_drone.block<3,1>(0,3) = dist;
			H_marker_drone(3,0) = 0.0;
			H_marker_drone(3,1) = 0.0;
			H_marker_drone(3,2) = 0.0;
			H_marker_drone(3,3) = 1.0;

			H_drone_odom.block<3,3>(0,0) = R_drone;
			H_drone_odom.block<4,1>(0,3) = world_pos_odom;
			H_drone_odom(3,0) = 0.0;
			H_drone_odom(3,1) = 0.0;
			H_drone_odom(3,2) = 0.0;

			H_drone_arena = H_marker_arena * H_marker_drone.inverse();

			H_odom_arena = H_drone_arena * H_drone_odom.inverse();

			Eigen::Matrix3d R_odom_arena;
			R_odom_arena = H_odom_arena.block<3,3>(0,0);
			Eigen::Vector3d XYZ_odom_arena;
			XYZ_odom_arena = utilities::R2XYZ( R_odom_arena );
			double yaw_odom_arena = XYZ_odom_arena(2);

			H_odom_arena.block<3,3>(0,0) = utilities::rotz(yaw_odom_arena);

			if( !correct_z){
				H_odom_arena(2,3) = nvg->getWorldTransform()(2,3);
			}
			
			//cout << "marker_pos: " << marker_pos.transpose() << " norm: " << marker_pos.norm() << endl;
			//cout << "dist: " << dist.transpose() << " norm: " << dist.norm() << endl;
			//cout << "world pos: " << nvg->getWorldPos().transpose() << endl;
			//cout << "odom_pos: " << nvg->getWorldPosOdom().transpose() << " norm: " << nvg->getWorldPosOdom().norm() << endl;
			//cout << "H_odom_arena: \n" << H_odom_arena << endl;
			//cout << "yaw_offset: " << utilities::R2XYZ( H_odom_arena.block<3,3>(0,0))(2) << endl;
			
			

			if( hard )
				nvg->setWorldTransform( H_odom_arena );
			else{
				//PROVA
				nvg->setWorldTransform( H_odom_arena );
				_H_odom_arena_sp =  H_odom_arena;
				_new_transform = true;
			}
			
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
		cout << "takeoff si \n";
		if( getNearestMarker(marker)){
			cout << "nearest: " << marker.id << endl;
			if(marker.id == id ){
				cout << "sono sul marker richiesto \n";
				return true;
			}
		}
		else if( getKnownMarkerPos(id, target)){
			cout << "Vado al marker richiesto \n";
			target(2) = height;
			cout << "marker target: " << target.transpose() << endl;
			nvg->moveTo(target);

			return true;
		}
	}
	else
		return false;
}

void ArucoManager::visualServoing(){
	
	aruco_msgs::Marker marker;
	Eigen::Vector3d dist_marker;
	Eigen::Vector4d att_marker;
	Eigen::Vector3d pos_drone_odom;

	Eigen::Vector3d pos_marker_odom;
	Eigen::Vector3d XYZ_marker_odom;

	Eigen::Matrix4d H_marker_drone;
	Eigen::Matrix4d H_drone_odom;
	Eigen::Matrix4d H_marker_odom;

	Eigen::Vector3d err;
	Eigen::Vector3d old_err;
	Eigen::Vector3d PD_out;
	Eigen::Vector3d target;
	Eigen::Vector3d old_target;

	Eigen::Vector3d des_vec;
	double cos_theta;
	double theta;
	double vel;

	ros::Rate r(_rate_servoing);
	double dt = 1.0/_rate_servoing;

	old_err << 0.0, 0.0, 0.0;
	err << 0.0, 0.0, 0.0;

	H_marker_drone = Eigen::Matrix4d::Identity();
	H_drone_odom   = Eigen::Matrix4d::Identity();
	H_marker_odom  = Eigen::Matrix4d::Identity();

	while(ros::ok()){
		
		if( _visual_servoing ){
			// Tutto calcolato in terna odom
			nvg->activateTrajectoryGenerator(false);
	
			if( getMarker(_servoing_marker_id, marker) && !_command_arrived ){

				

				cout << "visual servoing activated on: " << _servoing_marker_id << endl;
				
				// //Marker pos in drone frame
				// dist_marker << marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z;
				// att_marker << marker.pose.pose.orientation.w, marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z;

				// dist_marker = utilities::QuatToMat( att_marker ).transpose() * dist_marker;
				
				// // X-Y error
				// cout << "dist_marker: " << dist_marker.transpose() << endl;
				// pos_drone_odom = nvg->getWorldPosOdom().block<3,1>(0,0);
				// PD_out = _Kp_vs * dist_marker + _Kd_vs * ( ( dist_marker - old_err ) / dt ) ;
				// old_err = dist_marker;
				// cout << "PD_out: " << PD_out.transpose() << endl;
				// cout << "PD_out*dt: " << (PD_out*dt).transpose() << endl;

				// if( fabs(dist_marker(0)) > 0.05) 
				// 	target(0) = pos_drone_odom(0) + PD_out(0) * dt;
				// else
				// 	target(0) = old_target(0);

				// if( fabs(dist_marker(1)) > 0.05) 
				// 	target(1) = pos_drone_odom(1) + PD_out(1) * dt;
				// else
				// 	target(1) = old_target(1);

				//target(2) = 2.0;
				//nvg->moveTo(target);
				/*
				dist_marker << marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z;
				att_marker << marker.pose.pose.orientation.w, marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z;

				H_marker_drone.block<3,3>(0,0) = utilities::QuatToMat( att_marker );
				H_marker_drone.block<3,1>(0,3) = dist_marker;

				pos_drone_odom = nvg->getWorldPosOdom().block<3,1>(0,0);
				H_drone_odom.block<3,3>(0,0) = utilities::QuatToMat( nvg->getWorldQuatOdom() );
				H_drone_odom.block<3,1>(0,3) = pos_drone_odom;

				H_marker_odom = H_drone_odom * H_marker_drone;

				pos_marker_odom = H_marker_odom.block<3,1>(0,3);
				XYZ_marker_odom = utilities::R2XYZ( H_marker_odom.block<3,3>(0,0) );

				cout << "Marker pos odom: " << pos_marker_odom.transpose() << endl;

				//X-Y visual servoing
				err = pos_marker_odom - pos_drone_odom;
				PD_out = _Kp_vs * err + _Kd_vs * ( ( err - old_err ) / dt );

				if( PD_out.norm() > _norm_threshold ){

					PD_out = (PD_out / PD_out.norm() ) * _norm_threshold;

				}

				old_err = err;
				cout << "old_target: " << old_target.transpose() << endl;

				// if( fabs(err(0)) > 0.05 )
				// 	target(0) = pos_drone_odom(0) + PD_out(0) * dt;
				// else
				// 	target(0) = old_target(0);

				// if( fabs(err(1)) > 0.05 )
				// 	target(1) = pos_drone_odom(1) + PD_out(1) * dt;
				// else
				// 	target(1) = old_target(1);

				target = pos_drone_odom + PD_out * dt;

				cout << "Drone pos odom: " << pos_drone_odom.transpose() << endl;
				cout << "err: " << err.transpose() << endl;
				cout << "PD_out: " << PD_out.transpose() << endl;

				//Turn off trajectory planner
				nvg->activateTrajectoryGenerator(false);

				if( _land_on_marker ){

					//Evaluate landing speed depending on the position of the drone inside the cone
					des_vec << 0.0, 0.0, -1.0*dist_marker(2);

					cos_theta = des_vec.dot(err) / (des_vec.norm() * err.norm());
					theta = acos(cos_theta);
					//theta = 0.9;
					vel = (-_vel_max/_theta_max) * theta + _vel_max;
					
					cout << "Theta: " << theta << endl;
					
				}
				
				if( _land_on_marker && ( (-1.0*dist_marker(2) > _min_height) && (theta < _theta_max) ) ){

					//discesa
					target(2) = pos_drone_odom(2) - vel;
					cout << "target in discesa : " << target.transpose() << endl;
					cout << "vel discesa: " << vel << endl;
					nvg->moveTo(target);

				}
				else if( _land_on_marker &&  (-1.0*dist_marker(2) > _min_height) && (theta > _theta_max) ){

					//Risalgo
					target(2) = pos_drone_odom(2) + vel;
					cout << "target in risalita : " << target.transpose() << endl;
					cout << "vel risalita: " << -1.0*vel << endl;
					nvg->moveTo(target);

				}
				else if( _land_on_marker && ( (-1.0*dist_marker(2) <= _min_height) && (theta < _theta_max) ) ){

					//Atterro
					_visual_servoing = false;
					cout << "atterro \n";
					nvg->activateTrajectoryGenerator(true);
					nvg->land();

				}
				else if( !_land_on_marker ){

					//Rimango alla stessa altezza
					target(2) = 1.30;
					cout << "target : " << target.transpose() << endl;
					nvg->moveTo(target);
					cout << " \n ---- \n";
				}*/

				dist_marker << marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z;
				cout << "dist_marker: " << dist_marker.transpose() << endl;
				att_marker << marker.pose.pose.orientation.w, marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z;

				H_marker_drone.block<3,3>(0,0) = utilities::QuatToMat( att_marker );
				H_marker_drone.block<3,1>(0,3) = dist_marker;

				pos_drone_odom = nvg->getWorldPosOdom().block<3,1>(0,0);
				H_drone_odom.block<3,3>(0,0) = utilities::QuatToMat( nvg->getWorldQuatOdom() );
				H_drone_odom.block<3,1>(0,3) = pos_drone_odom;

				H_marker_odom = H_drone_odom * H_marker_drone;

				pos_marker_odom = H_marker_odom.block<3,1>(0,3);
				XYZ_marker_odom = utilities::R2XYZ( H_marker_odom.block<3,3>(0,0) );

				//cout << "Marker pos odom: " << pos_marker_odom.transpose() << endl;

				//Turn off trajectory planner
				nvg->activateTrajectoryGenerator(false);

				//X-Y visual servoing
				err = pos_marker_odom - pos_drone_odom;
				PD_out = _Kp_vs * err + _Kd_vs * ( ( err - old_err ) / dt ) ;
				old_err = err;

				// if( PD_out.norm() > _norm_threshold ){

				// 	PD_out = (PD_out / PD_out.norm() ) * _norm_threshold;

				// }

				target = pos_drone_odom + PD_out * dt;

				// if( fabs(err(0)) > 0.03 )
				// 	target(0) = pos_drone_odom(0) + PD_out(0) * dt;
				// else
				// 	target(0) = old_target(0);

				// if( fabs(err(1)) > 0.03 )
				// 	target(1) = pos_drone_odom(1) + PD_out(1) * dt;
				// else
				// 	target(1) = old_target(1);

				//cout << "Drone pos odom: " << pos_drone_odom.transpose() << endl;
				cout << "err: " << err.transpose() << endl;
				//cout << "PD_out: " << PD_out.transpose() << endl;
				cout << "land: " << _land_on_marker << endl;
				if( _land_on_marker ){

					// des_vec << 0.0, 0.0, -1.0*dist_marker(2);

					// cos_theta = des_vec.dot(err) / (des_vec.norm() * err.norm());
					// theta = acos(cos_theta);
					// //theta = 0.9;
					// vel = (-_vel_max/_theta_max) * theta + _vel_max;

					// cout << "Theta: " << theta << endl;

					if( fabs(err(0)) <= 0.15 && fabs(err(1)) <= 0.15 )
						vel = 0.4;
					else
						vel = 0.0;
					
				}

				nvg->resetInterrupt();
				if( _land_on_marker && ( (-1.0*dist_marker(2) > _min_height) && (theta < _theta_max) ) ){

					//discesa
					target(2) = old_target(2) - vel * dt;
					cout << "target in discesa : " << target.transpose() << endl;
					cout << "vel discesa: " << vel << endl;
					nvg->moveTo(target);

				}
				else if( _land_on_marker && ( (-1.0*dist_marker(2) > _min_height) && (theta > _theta_max) ) ){

					//Risalgo
					target(2) = old_target(2) + vel * dt;
					cout << "target in risalita : " << target.transpose() << endl;
					cout << "vel risalita: " << vel << endl;
					nvg->moveTo(target);

				}
				else if( _land_on_marker && ( (-1.0*dist_marker(2) <= _min_height) && (theta < _theta_max) ) ){

					//Atterro
					_visual_servoing = false;
					cout << "atterro \n";
					nvg->activateTrajectoryGenerator(true);
					nvg->land();

				}
				else if( !_land_on_marker ){

					//Rimango alla stessa altezza
					target(2) = 1.30;
					cout << "target : " << target.transpose() << endl;
					nvg->moveTo(target);
					cout << " \n ---- \n";
				}
					

			}
			else{
				// cout << "Target PERSO \n";
				nvg->activateTrajectoryGenerator(true);
				nvg->interruptAll();

				if( !_land_on_marker ){

					while( !_command_arrived )
						sleep(0.1);
					_command_arrived = false;
					cout << "comando ricevuto \n";
					_visual_servoing = false;
					nvg->resetInterrupt();
				}
				else{

					target = nvg->getWorldPos().block<3,1>(0,0);

					if( target(2) <= 2.0 )
						target(2) += 0.20;
					else
						target(2) = 2.20;

					_visual_servoing = false;

					nvg->resetInterrupt();
					nvg->moveTo( nvg->getWorldPos().block<3,1>(0,0) );
				}
				
			}
			old_target = target;
			
		}
		else{
			nvg->activateTrajectoryGenerator(true);
			// nvg->moveTo( nvg->getWorldPos().block<3,1>(0,0) );
			old_target = nvg->getWorldPosOdom().block<3,1>(0,0);
			// old_err = Eigen::Vector3d::Zero();
		}

		/** TO_DO: inserire i casi in cui viene perso di vista il marker */
				
		r.sleep();
	}

}

bool ArucoManager::landOnMarker(const int id){
	aruco_msgs::Marker marker;
	Eigen::Vector3d target;

	if( nvg->getTakeoff() ){

		if( getNearestMarker(marker) && marker.id == id ){

			cout << "sto già sul marker \n";

			_servoing_marker_id = id;
			_land_on_marker = true;
			_visual_servoing = true;

			while( !nvg->getLand() )
				sleep(0.01);

			return true;

		}
		else if( getKnownMarkerPos(id, target) && nvg->getWorldPos()(2) > _min_height){

			cout << "vado al marker \n";

			moveToMarker(id, 2.0); /** IMPOSTARE ALTEZZA **/
			sleep(0.5);

			cout << "attivo visual servoing \n";
			_servoing_marker_id = id;
			_land_on_marker = true;
			_visual_servoing = true;

			while( !nvg->getLand() )
				sleep(0.01);

			return true;
		}
		else
			return false;
	}
}


void ArucoManager::worldTransformFilter(){
	
	Eigen::Matrix4d H_odom_arena;
	Eigen::Matrix4d actual_H_odom_arena;
	Eigen::Matrix4d H_error;
	Eigen::Vector3d actual_H_pos;
	Eigen::Vector3d new_H_pos;
	double norm_err_pos = 0.0;


	ros::Rate r(_transform_filter_rate);
	double dt = 1.0 / _transform_filter_rate;

	H_odom_arena = Eigen::Matrix4d::Identity();

	while( ros::ok() ){	

		if( _new_transform ){
			actual_H_odom_arena = nvg->getWorldTransform();
			actual_H_pos = actual_H_odom_arena.block<3,1>(0,3);
			new_H_pos = _H_odom_arena_sp.block<3,1>(0,3);

			H_error = _H_odom_arena_sp.transpose() * actual_H_odom_arena;
			norm_err_pos = (new_H_pos - actual_H_pos).norm();


			if( norm_err_pos <= 1.0 && norm_err_pos > 0.05 ){

				new_H_pos = actual_H_pos + ( new_H_pos - actual_H_pos )/dt;

				H_odom_arena.block<3,3>(0,0) = _H_odom_arena_sp.block<3,3>(0,0);
				H_odom_arena.block<3,1>(0,3) = new_H_pos;

				nvg->setWorldTransform( H_odom_arena);
			
			}
			else if( norm_err_pos <= 0.05 )
				_new_transform = false;
			else if( norm_err_pos > 1.0 )
				cout << "Trasformazione troppo differente, norm: " << norm_err_pos << endl;
			
		}


		r.sleep();
	}

}

void ArucoManager::localize(bool hard, bool correct_z){
	ros::Rate r(10);
	int count = 0;
	Eigen::Vector3d av, di, dx, sx;
	av << 0.5, 0.0, 0.0;
	di << -0.5, 0.0, 0.0;
	dx << 0.0, -0.5, 0.0;
	sx << 0.0, 0.5, 0.0;

	if( !nvg->getInterrupt() ){

		nvg->setLocalizationStatus(false);

		while( !nvg->getLocalizationStatus() && count <5 ){
			correctWorldTransform(hard, correct_z);
			count ++;
			r.sleep();
		}
		if( !nvg->getLocalizationStatus() ){
			cout << "Non localizzato, avanti! \n";
			nvg->moveTo( nvg->getWorldPos().block<3,1>(0,0) + av );
		}

		while( !nvg->getLocalizationStatus() && count <5 ){
			correctWorldTransform(hard, correct_z);
			count ++;
			r.sleep();
		}
		if( !nvg->getLocalizationStatus() ){
			cout << "Non localizzato, dietro! \n";
			nvg->moveTo( nvg->getWorldPos().block<3,1>(0,0) -av +di );
		}

		while( !nvg->getLocalizationStatus() && count <5 ){
			correctWorldTransform(hard, correct_z);
			count ++;
			r.sleep();
		}
		if( !nvg->getLocalizationStatus() ){
			cout << "Non localizzato, sx! \n";
			nvg->moveTo( nvg->getWorldPos().block<3,1>(0,0) -di +sx );
		}

		while( !nvg->getLocalizationStatus() && count <5 ){
			correctWorldTransform(hard, correct_z);
			count ++;
			r.sleep();
		}
		if( !nvg->getLocalizationStatus() ){
			cout << "Non localizzato, dx! \n";
			nvg->moveTo( nvg->getWorldPos().block<3,1>(0,0) -sx +dx );
		}

		if( !nvg->getLocalizationStatus() ){
			cout << "Non localizzato, atterro! \n";
			nvg->land();
		}
	}

}

void ArucoManager::Routine(){
	aruco_msgs::Marker mark;
	Eigen::Vector3d pos;
	bool landed = false;
	bool moved = false;
	bool first_takeoff = false;
	string key;
	Eigen::MatrixXd wps;
	int takeoff_id;
	int rs;
	bool seq_finished = false;

	ros::Rate r(10);

	_des_height = 2.0;

	_continous_correction = false;

	getline(cin, key);

	if(key == "p"){
		while( ros::ok() ){
			
			if( !first_takeoff ){
				cout << "TAKEOFF \n";
				nvg->takeoff(_des_height, 0.3);
				localize(true, true);
				getNearestKnownMarker(mark);
				takeoff_id = mark.id;
				
				if( nvg->getLocalizationStatus() )
					first_takeoff = true;
				sleep(0.1);
				
				if( takeoff_id != 3 ){
						wps = load_wps(takeoff_id,3);
						nvg->moveToWps( wps );
						sleep(2.0);
						localize(true, _corr_z);
				}
				
			}
			else{
				
				if(!_atterraggi && !nvg->getInterrupt() && !_visual_servoing ){
					cout << "Sequenza \n";

					double min_pos_mark = 10000;
					int near_id;
					int vec[3] = {3, 6, 9};

					for(int j=0; j<3; j++){
						int norm;
						Eigen::Vector3d pos_mark;
						Eigen::Vector3d pos_drone;

						int i = vec[j]-1;

						pos << _knownList[i].pose.pose.position.x, _knownList[i].pose.pose.position.y, _knownList[i].pose.pose.position.z;
						pos_drone  = nvg->getWorldPos().block<3,1>(0,0);


						norm = (pos - pos_drone).norm();

						if( min_pos_mark > norm ){
							min_pos_mark = norm;
							near_id = _knownList[i].id;
						}
					}
					
					moveToMarker(near_id, 2.00);
					sleep(2.0);
					localize(true, _corr_z);

					if( near_id == 3 )	
						rs = 4;
					else if(near_id == 6)
						rs = 5;
					else
						rs = 6;

					while( !nvg->getInterrupt() && !_visual_servoing ){
						wps = load_research(rs);
						nvg->moveToWps( wps );
						sleep(2.0);
						localize(true, _corr_z);
						
						rs++;
						if( rs>6 )
							rs = 1;
					}			
					
				}
				else if( _atterraggi && !nvg->getInterrupt() ){

					double min_pos_mark = 10000;
					int near_id;

					nvg->resetInterrupt();

					for(int i=0; i<10; i++){
						int norm;
						Eigen::Vector3d pos_mark;
						Eigen::Vector3d pos_drone;

						pos << _knownList[i].pose.pose.position.x, _knownList[i].pose.pose.position.y, _knownList[i].pose.pose.position.z;
						pos_drone  = nvg->getWorldPos().block<3,1>(0,0);


						norm = (pos - pos_drone).norm();

						if( min_pos_mark > norm ){
							min_pos_mark = norm;
							near_id = _knownList[i].id;
						}
					}
					cout << "vado a: " << near_id << endl;
					moveToMarker(near_id, 2.10);
					sleep(0.5);

					int k = 0;
					bool present = false;

					while( k < _sequenza.data.size() && !present ){

						if( _sequenza.data[k] == near_id )
							present = true;
						else
							k++;
					}

					if( present ){
						landOnMarker(near_id);
						_sequenza.data.erase( _sequenza.data.begin() + k );
						sleep(0.2);
						nvg->takeoff(2.0, 0.3);
						sleep(1.5);
						localize(true, _corr_z);
					}
					else if(!seq_finished){

						wps = load_wps(near_id,_sequenza.data[0]);
						nvg->moveToWps( wps );
						sleep(0.5);
						landOnMarker( _sequenza.data[0] );
						sleep(0.2);

						for(int j=1; j<_sequenza.data.size(); j++){

							nvg->takeoff(2.0, 0.3);
							sleep(1.0);
							localize(true, _corr_z);

							wps = load_wps( _sequenza.data[j-1], _sequenza.data[j] );
							nvg->moveToWps( wps );
							sleep(0.2);
							landOnMarker( _sequenza.data[j] );
							sleep(0.2);
							
						}

						seq_finished = true;

					}
					
					
				}
				
				
			}
			r.sleep();
		}
	}
}


void ArucoManager::run(){

	boost::thread traj_gen_t( &Navigation::setPointPublisher, nvg);
	sleep(1);
	//boost::thread transform_filter_t( &ArucoManager::worldTransformFilter, this);
	boost::thread visual_t( &ArucoManager::visualServoing, this);
	boost::thread routine_t( &ArucoManager::Routine, this);

	//boost::thread tf_broadcast_poses_t( &Navigation::tf_broadcast_poses, nvg);
	//boost::thread tf_broadcast_pose_odom_t( &Navigation::tf_broadcast_pose_odom, nvg);
	//boost::thread tf_broadcast_pose_arena_t( &Navigation::tf_broadcast_pose_arena, nvg);
	//boost::thread tf_broadcast_odom_arena_t( &Navigation::tf_broadcast_odom_arena, nvg);
	
	
}


