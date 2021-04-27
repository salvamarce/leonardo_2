#include "navigation.h"

Navigation::Navigation() {

   if( !_nh.getParam("pos_threshold", _pos_threshold)) {
      _pos_threshold = 0.03;
   }

   if( !_nh.getParam("yaw_threshold", _yaw_threshold)) {
      _yaw_threshold = 0.2;
   }

   if( !_nh.getParam("height_threshold", _height_threshold)) {
      _height_threshold = 0.2;
   }

   if( !_nh.getParam("max_cruise_vel", _cruise_vel)) {
      _cruise_vel = 3.0;
   }

   if( !_nh.getParam("trajectory_rate", _traj_rate)) {
      _traj_rate = 20.0;
   }

   if( !_nh.getParam("setpoint_topic", _setpoint_topic)) {
      _setpoint_topic = "/mavros/setpoint_raw/local";
   }

   if( !_nh.getParam("pose_topic", _pose_topic)) {
      _pose_topic = "/mavros/local_position/pose";
   }

   if( !_nh.getParam("mavros_state_topic", _mavros_state_topic)) {
      _mavros_state_topic = "/mavros/state";
   }

	_first_local_pos = false;
   _setpoint_pub = _nh.advertise<mavros_msgs::PositionTarget>( _setpoint_topic.c_str() , 1);
   _pose_sub = _nh.subscribe( _pose_topic.c_str() , 1, &Navigation::pose_cb, this);
   _mavros_state_sub = _nh.subscribe( _mavros_state_topic.c_str(), 1, &Navigation::mavros_state_cb, this);

   // --- Services ---
   _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
   _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
   _land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
   //---

   // --- Trajectory ---
   _trajectory = new CARTESIAN_PLANNER(_traj_rate);
   _traj_finished = true;
   _take_off = false;
   _act_traj_gen = true;

   _world_offset << 0.0, 0.0, 0.0;

}

void Navigation::mavros_state_cb( mavros_msgs::State mstate) {
   _mstate = mstate;
}

void Navigation::pose_cb ( geometry_msgs::PoseStampedConstPtr msg ) {

   _world_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
   
   _world_pos += _world_offset;

   Eigen::Vector3d rpy = utilities::R2XYZ ( utilities::QuatToMat ( Eigen::Vector4d( msg->pose.orientation.w,  msg->pose.orientation.x,  msg->pose.orientation.y,  msg->pose.orientation.z) ) );
   _mes_yaw = rpy(2);

   Quaternionf q;
   q = AngleAxisf(0.0, Vector3f::UnitX())
      * AngleAxisf(0.0, Vector3f::UnitY())
      * AngleAxisf(_mes_yaw, Vector3f::UnitZ());
   Vector4d w_q ( q.w(), q.x(), q.y(), q.z() );
   _world_quat = w_q / w_q.norm() ;

   _first_local_pos = true;

}

void Navigation::takeoff( const double altitude, double vel) {
  
  ros::Rate rate(10);
  
  //Set up control mode
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  //---

  if( _set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
    ROS_INFO("OFFBOARD mode enabled");
  }

  //---Arm
  if( _arming_client.call(arm_cmd) && arm_cmd.response.success){
  }

  while(!_mstate.armed ) usleep(0.1*1e6);
  ROS_INFO("Vehicle armed");
  //---

  while(_mstate.mode != "OFFBOARD" ) usleep(0.1*1e6);
  ROS_INFO("Vehicle in offboard");

  if(vel != 0.0)
      _traj_vel = vel;
   else
      _traj_vel = _cruise_vel;
   
   _setpoint_pos(2) = altitude;
  
  while( !arrived() ) {
    usleep(0.1*1e5);
  }

  ROS_INFO("Takeoff completed");
  _take_off = true;
  
}

void Navigation::rotate( const double angle, double vel) {
  
  if(vel != 0.0)
      _traj_vel = vel;
   else
      _traj_vel = _cruise_vel;

   _setpoint_yaw  = angle;
}

bool Navigation::arrived(){
   double epx = _setpoint_pos(0) - _world_pos(0);
   double epy = _setpoint_pos(1) - _world_pos(1);
   double epz = _setpoint_pos(2) - _world_pos(2);
   double epyaw = _setpoint_yaw - _mes_yaw;

   //cout << "sp: " << _setpoint_pos.transpose() << endl;
   //cout << "wp: " << _world_pos.transpose() << endl;
   //cout << "ep: " << epx << " " << epy << " " << epz << endl;

   if( (epx < _pos_threshold && epx > -1.0*_pos_threshold ) &&
       (epy < _pos_threshold && epy > -1.0*_pos_threshold ) &&
       (epz < _height_threshold && epz > -1.0*_height_threshold ) &&
       (epyaw < _yaw_threshold && epyaw > -1.0*_yaw_threshold ) ){
      
      _traj_finished = true;
      return true;
   }
   else
      return false;

}

void Navigation::trajectoryGenerator(){
   
   ros::Rate r(_traj_rate);
   double time_traj;
   double time_traj_pos, time_traj_yaw;
   std::vector<double> times;
   geometry_msgs::PoseStamped pose_sp;
   std::vector<geometry_msgs::PoseStamped> poses;
   Eigen::Vector4d quat;
   Eigen::Vector3d XYZ;
   Eigen::Vector3d old_sp_pos;
   double old_sp_yaw;

   geometry_msgs::PoseStamped x_traj;
   geometry_msgs::TwistStamped xd_traj;
   geometry_msgs::AccelStamped xdd_traj;

   Eigen::Vector3d des_pos;
   Eigen::Vector3d des_vel;
   Eigen::Vector3d des_acc;
   Eigen::Vector4d des_att;
   Eigen::Vector3d des_ang_vel;

   mavros_msgs::PositionTarget ptarget;
   ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
   ptarget.type_mask =
   mavros_msgs::PositionTarget::IGNORE_VX |
   mavros_msgs::PositionTarget::IGNORE_VY |
   mavros_msgs::PositionTarget::IGNORE_VZ |
   mavros_msgs::PositionTarget::IGNORE_AFX |
   mavros_msgs::PositionTarget::IGNORE_AFY |
   mavros_msgs::PositionTarget::IGNORE_AFZ |
   mavros_msgs::PositionTarget::FORCE |
   mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

   while( !_first_local_pos )
      usleep(0.1*1e6);
   ROS_INFO("First local pose arrived!");

   des_pos = old_sp_pos = _setpoint_pos = _world_pos;
   old_sp_yaw = _setpoint_yaw = _mes_yaw;
   XYZ << 0.0, 0.0, _mes_yaw;
   des_att = utilities::rot2quat( utilities::XYZ2R(XYZ) );

   while (ros::ok()) {
      
      if( _mstate.mode != "OFFBOARD") {
         des_pos = old_sp_pos = _setpoint_pos = _world_pos;
         old_sp_yaw = _setpoint_yaw = _mes_yaw;
      }
      else if( _act_traj_gen ){
         if( old_sp_yaw != _setpoint_yaw || old_sp_pos != _setpoint_pos ){

            poses.clear();
            times.clear();
            
            pose_sp.pose.position.x = _world_pos(0) - _world_offset(0);
            pose_sp.pose.position.y = _world_pos(1) - _world_offset(1);
            pose_sp.pose.position.z = _world_pos(2) - _world_offset(2);

            XYZ << 0.0, 0.0, _mes_yaw;
            quat = utilities::rot2quat( utilities::XYZ2R(XYZ) );
            pose_sp.pose.orientation.w = quat(0);
            pose_sp.pose.orientation.x = quat(1);
            pose_sp.pose.orientation.y = quat(2);
            pose_sp.pose.orientation.z = quat(3);

            poses.push_back(pose_sp);
            times.push_back(0.0);
            
            //_setpoint_pos -= _world_offset;

            if( old_sp_pos != _setpoint_pos )
               time_traj_pos = (_setpoint_pos - _world_pos).norm() / _traj_vel;
            else
               time_traj_pos = 0.0;

            
            if( old_sp_yaw != _setpoint_yaw )
               time_traj_yaw = (_setpoint_yaw - _mes_yaw) / _traj_vel;
            else
               time_traj_yaw = 0.0;


            if( time_traj_pos >= time_traj_yaw)
               time_traj = time_traj_pos;
            else 
               time_traj = time_traj_yaw;

            times.push_back(time_traj);

            pose_sp.pose.position.x = _setpoint_pos(0) - _world_offset(0);
            pose_sp.pose.position.y = _setpoint_pos(1) - _world_offset(1);
            pose_sp.pose.position.z = _setpoint_pos(2) - _world_offset(2);

            XYZ << 0.0, 0.0, _setpoint_yaw;
            quat = utilities::rot2quat( utilities::XYZ2R(XYZ) );
            pose_sp.pose.orientation.w = quat(0);
            pose_sp.pose.orientation.x = quat(1);
            pose_sp.pose.orientation.y = quat(2);
            pose_sp.pose.orientation.z = quat(3);

            poses.push_back(pose_sp);
            _trajectory->set_waypoints(poses, times);
            _traj_finished = false;
            _trajectory->compute();
            old_sp_pos = _setpoint_pos;
            old_sp_yaw = _setpoint_yaw;
         }
         else{
            if( !arrived() && _trajectory->getNext(x_traj, xd_traj, xdd_traj) ) { 
                  des_pos << x_traj.pose.position.x, x_traj.pose.position.y, x_traj.pose.position.z;
                  des_vel << xd_traj.twist.linear.x, xd_traj.twist.linear.y, xd_traj.twist.linear.z;
                  des_acc << xdd_traj.accel.linear.x, xdd_traj.accel.linear.y, xdd_traj.accel.linear.z;
                  des_att << x_traj.pose.orientation.w, x_traj.pose.orientation.x, x_traj.pose.orientation.y, x_traj.pose.orientation.z;
                  des_ang_vel << xd_traj.twist.angular.x, xd_traj.twist.angular.y, xd_traj.twist.angular.z;
            }
         }
      }
      else if( !_act_traj_gen ){
         des_pos = old_sp_pos = _setpoint_pos;
         old_sp_yaw = _setpoint_yaw;
         XYZ << 0.0, 0.0, _setpoint_yaw;
         des_att = utilities::rot2quat( utilities::XYZ2R(XYZ) );
      }
      //---Publish command
      ptarget.header.stamp = ros::Time::now();
      ptarget.position.x = des_pos[0];
      ptarget.position.y = des_pos[1];
      ptarget.position.z = des_pos[2];
      XYZ = utilities::R2XYZ( utilities::QuatToMat(des_att) );
      ptarget.yaw = XYZ(2);
      _setpoint_pub.publish( ptarget );
      //---
      r.sleep();
      
   }
}

void Navigation::move_to( const Eigen::Vector3d dest, const double yaw, const double vel ) {

   if(vel != 0.0)
      _traj_vel = vel;
   else
      _traj_vel = _cruise_vel;
   
   _setpoint_pos = dest;

   if(yaw != 0.0)
      _setpoint_yaw = yaw;
   else
      _setpoint_yaw = _mes_yaw;

}

void Navigation::land( double altitude, double vel) {  

   if(vel != 0.0)
      _traj_vel = vel;
   else
      _traj_vel = _cruise_vel;

   _setpoint_pos(0) = _world_pos(0);
   _setpoint_pos(1) = _world_pos(1);
   _setpoint_pos(2) = altitude - _height_threshold;

   cout << "Waiting land" << endl;
   while( !arrived() ) {
      cout << "world: " << _world_pos.transpose() << endl;
      cout << "setpoint: " << _setpoint_pos.transpose() << endl;
      usleep(0.1*1e6);
   }

   mavros_msgs::CommandTOL land_srv;
   _land_client.call( land_srv );
   while( _mstate.armed ) usleep(0.1*1e6);
   cout << "Disarmed!" << endl;

}

void Navigation::setWorldOffset(const Eigen::Vector3d &offset){ 
   
   if( offset != _world_offset){
      _world_offset = offset;
      _world_pos += _world_offset;
   }
}

void Navigation::select_action() {

  string line;
  ros::Rate r(1);

  while(ros::ok()) {
      
      cout << "--------------------" << endl;
      cout << "Insert new action: " << endl;
      cout << "1 - takeoff" << endl;
      cout << "2 - land" << endl;
      cout << "3 - rotate" << endl;
      cout << "4 - move" << endl;
      cout << "--------------------" << endl;
      
      getline(cin, line);
      

      if( line == "1" ) {
         takeoff(1.8, 5.0);
      }
      else if( line == "2") {
         land(0.0, 2.0);
      }
      else if( line == "3" ) {
         cout << "Insert desired angle" << endl;
         getline(cin, line);
         rotate( stod(line), 0.0);
      }
      else if( line == "4" ) {
         cout << "Actual position: " << _world_pos.transpose() << " yaw: " << _mes_yaw << endl;
         cout << "Insert destination point" << endl;
         float x, y, z;
         scanf("%f %f %f", &x, &y, &z );

         Vector3d dest;
         dest = Vector3d ( x, y, z );
         move_to(dest);
      }
      
      r.sleep();
   }
}

void Navigation::run(){
   boost::thread trajectoryGenerator_t( &Navigation::trajectoryGenerator, this);

   if( _test_mode )
      boost::thread select_action_t( &Navigation::select_action, this );

   //ros::spin();
}


