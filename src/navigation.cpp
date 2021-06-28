#include "navigation.h"

Navigation::Navigation() {
   double traj_pos_thr_x, traj_pos_thr_y, traj_pos_thr_z;

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

   if( !_nh.getParam("max_height", _max_height)) {
      _max_height = 3.5;
   }

   if( !_nh.getParam("drone_height", _drone_height)) {
      _drone_height = 0.21;
   }

   if( !_nh.getParam("trajectory_rate", _traj_rate)) {
      _traj_rate = 20.0;
   }

   if( !_nh.getParam("sp_rate", _sp_rate)) {
      _sp_rate = 50.0;
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
   _take_off = false;
   _act_traj_gen = true;

   _H_odom_arena << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;

   _world_pos << 0.0, 0.0, 0.0, 1.0;

}

void Navigation::mavros_state_cb( mavros_msgs::State mstate) {
   _mstate = mstate;
}

void Navigation::pose_cb ( geometry_msgs::PoseStamped msg ) {

   Eigen::Vector4d temp;
   Eigen::Vector3d rpy;

   _world_pos_odom << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z + _drone_height, 1.0;

   _world_pos = _H_odom_arena * _world_pos_odom;

   rpy = utilities::R2XYZ ( _H_odom_arena.block<3,3>(0,0) * utilities::QuatToMat ( Eigen::Vector4d( msg.pose.orientation.w,  msg.pose.orientation.x,  msg.pose.orientation.y,  msg.pose.orientation.z) ) );
   _world_quat = utilities::rot2quat( _H_odom_arena.block<3,3>(0,0) * utilities::QuatToMat ( Eigen::Vector4d( msg.pose.orientation.w,  msg.pose.orientation.x,  msg.pose.orientation.y,  msg.pose.orientation.z) )  );
   _mes_yaw = rpy(2);

   rpy = utilities::R2XYZ ( utilities::QuatToMat ( Eigen::Vector4d( msg.pose.orientation.w,  msg.pose.orientation.x,  msg.pose.orientation.y,  msg.pose.orientation.z) ) );
   _mes_yaw_odom = rpy(2); 

   _world_quat_odom << msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z;

   _first_local_pos = true;

}

void Navigation::takeoff( const double altitude, double vel) {
  
   ros::Rate rate(10);
   Eigen::Vector3d pos_sp;

   pos_sp << _world_pos(0), _world_pos(1), altitude;

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

   moveTo(pos_sp, vel);

   while( !arrived( pos_sp ) ) {
      sleep(0.001);
   }

   ROS_INFO("Takeoff completed");
   _take_off = true;

}

void Navigation::rotate( const double angle, double vel) {
  
   moveToWithYaw(_world_pos.block<3,1>(0,0), angle, vel);

}

bool Navigation::arrived( const Eigen::Vector3d pos_sp, const double yaw_sp ){
   
   double epx, epy, epz, epyaw;  

   if (pos_sp(0) >= 0)
      epx = pos_sp(0) - _world_pos(0);
   else
      epx = pos_sp(0) + _world_pos(0);
   
   if (pos_sp(1) >= 0)
      epy = pos_sp(1) - _world_pos(1);
   else
      epy = pos_sp(1) + _world_pos(1);

   if (pos_sp(2) >= 0)
      epz = pos_sp(2) - _world_pos(2);
   else
      epz = pos_sp(2) + _world_pos(2);
   
   if (yaw_sp >= 0)
      epyaw = yaw_sp - _mes_yaw;
   else
      epyaw = yaw_sp + _mes_yaw;
  

   if( (epx < _pos_threshold && epx > -1.0*_pos_threshold ) &&
       (epy < _pos_threshold && epy > -1.0*_pos_threshold ) &&
       (epz < _height_threshold && epz > -1.0*_height_threshold ) &&
       (epyaw < _yaw_threshold && epyaw > -1.0*_yaw_threshold ) )
   {
      
      //cout << "pos e yaw arrivati \n";
      return true;
   }
   else{
      //cout << "pos e yaw non arrivati " << epx << " " << epy << " " << epz << "   yaw: " << yaw_sp << endl;
      return false;
   }

}

bool Navigation::arrived( const Eigen::Vector3d pos_sp){
   
   double epx, epy, epz;  

   if (pos_sp(0) >= 0)
      epx = pos_sp(0) - _world_pos(0);
   else
      epx = pos_sp(0) + _world_pos(0);
   
   if (pos_sp(1) >= 0)
      epy = pos_sp(1) - _world_pos(1);
   else
      epy = pos_sp(1) + _world_pos(1);

   if (pos_sp(2) >= 0)
      epz = pos_sp(2) - _world_pos(2);
   else
      epz = pos_sp(2) + _world_pos(2);

   if( (epx < _pos_threshold && epx > -1.0*_pos_threshold ) &&
       (epy < _pos_threshold && epy > -1.0*_pos_threshold ) &&
       (epz < _height_threshold && epz > -1.0*_height_threshold ) )
   {
   
      return true;
   }
   else{

      return false;
   }

}

bool Navigation::arrived( const double yaw_sp ){
   
   double epyaw;  

   if (yaw_sp >= 0)
      epyaw = yaw_sp - _mes_yaw;
   else
      epyaw = yaw_sp + _mes_yaw;

   if( epyaw < _yaw_threshold && epyaw > -1.0*_yaw_threshold  )
   {
      
      return true;
   }
   else{

      return false;
   }

}

void Navigation::setPointPublisher(){

   ros::Rate r(_sp_rate);

   Eigen::Vector3d des_pos;
   Eigen::Vector4d des_att;
   double des_yaw;
   
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

    // *** Ragiona sempre in terna odom ***

   des_pos  = (_H_odom_arena.inverse() * _world_pos).block<3,1>(0,0);
   des_yaw = _mes_yaw;

   while( ros::ok() ){

      if( _mstate.mode != "OFFBOARD") {
         
         des_pos  = _world_pos_odom.block<3,1>(0,0);
         des_yaw = _mes_yaw_odom;
      }
      else if( isnan( _pos_sp(0) ) || isnan( _pos_sp(1) ) || isnan( _pos_sp(2) ) || isnan( _yaw_sp ) ){

         cout << "pos_sp is NaN " << _pos_sp.transpose() << " yaw: " << _yaw_sp << endl;
         des_pos  = _world_pos_odom.block<3,1>(0,0);
         des_yaw = _mes_yaw_odom;

      }
      else{

         
         //des_pos = _H_odom_arena.block<3,3>(0,0) * _pos_sp;
         des_pos = _pos_sp;

         if(des_pos(2) < 0.0) 
            des_pos(2) = 0.0;
         else if(des_pos(2) > _max_height)
            des_pos(2) = _max_height;

         des_yaw = _yaw_sp;

      }
      
      ptarget.header.stamp = ros::Time::now();
      ptarget.position.x = des_pos(0);
      ptarget.position.y = des_pos(1);
      ptarget.position.z = des_pos(2);         
      ptarget.yaw = des_yaw ;
      
      _setpoint_pub.publish( ptarget );
     
      r.sleep();
   }

}

void Navigation::trajectoryGenerator(const Eigen::Vector3d pos, const double yaw, const double vel){

   std::vector<double> times;
   std::vector<geometry_msgs::PoseStamped> poses;
   geometry_msgs::PoseStamped pose_sp;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d quat;
   double time_traj_pos, time_traj_yaw;

   poses.clear();
   times.clear();

   pose_sp.pose.position.x = _world_pos(0);
   pose_sp.pose.position.y = _world_pos(1);
   pose_sp.pose.position.z = _world_pos(2);

   XYZ << 0.0, 0.0, _mes_yaw;
   quat = utilities::rot2quat( utilities::XYZ2R(XYZ) );
   pose_sp.pose.orientation.w = quat(0);
   pose_sp.pose.orientation.x = quat(1);
   pose_sp.pose.orientation.y = quat(2);
   pose_sp.pose.orientation.z = quat(3);

   poses.push_back(pose_sp);
   times.push_back(0.0);
   
   if( pos != _world_pos.block<3,1>(0,0) ){
      time_traj_pos = (pos - _world_pos.block<3,1>(0,0)).norm() / vel;
   }
   else
      time_traj_pos = 0.1;

   if( yaw != _mes_yaw ){
      if( yaw > _mes_yaw )
         time_traj_yaw = ( yaw - _mes_yaw) / vel;
      else if( yaw < _mes_yaw )
         time_traj_yaw = ( _mes_yaw - yaw ) / vel;
   }
   else
      time_traj_yaw = 0.1;

   if( time_traj_pos >= time_traj_yaw )
      times.push_back(time_traj_pos);
   else
      times.push_back(time_traj_yaw);

   pose_sp.pose.position.x = pos(0);
   pose_sp.pose.position.y = pos(1);
   pose_sp.pose.position.z = pos(2);

   XYZ << 0.0, 0.0, yaw;
   quat = utilities::rot2quat( utilities::XYZ2R(XYZ) );
   pose_sp.pose.orientation.w = quat(0);
   pose_sp.pose.orientation.x = quat(1);
   pose_sp.pose.orientation.y = quat(2);
   pose_sp.pose.orientation.z = quat(3);

   poses.push_back(pose_sp);

   _trajectory->set_waypoints(poses, times);    

   _trajectory->compute();
   cout << "traiettoria calcolata \n";

}

void Navigation::trajectoryGeneratorWps(const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> pos, const Eigen::Ref<Eigen::VectorXd> yaw , const double vel){
   
   std::vector<double> times;
   std::vector<geometry_msgs::PoseStamped> poses;
   geometry_msgs::PoseStamped pose_sp;
   Eigen::Vector3d old_sp_pos;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d quat;
   double old_sp_yaw;
   double time_traj;
   double time_traj_pos, time_traj_yaw;
   int wps_number = 0;

   poses.clear();
   times.clear();

   pose_sp.pose.position.x = _world_pos(0);
   pose_sp.pose.position.y = _world_pos(1);
   pose_sp.pose.position.z = _world_pos(2);

   XYZ << 0.0, 0.0, _mes_yaw;
   quat = utilities::rot2quat( utilities::XYZ2R(XYZ) );
   pose_sp.pose.orientation.w = quat(0);
   pose_sp.pose.orientation.x = quat(1);
   pose_sp.pose.orientation.y = quat(2);
   pose_sp.pose.orientation.z = quat(3);

   poses.push_back(pose_sp);
   times.push_back(0.0);

   old_sp_pos = _world_pos.block<3,1>(0,0);
   old_sp_yaw = _mes_yaw;
            
   if( pos.cols() > yaw.rows())
      wps_number = pos.cols();
   else
      wps_number = yaw.rows();

   for(int i=0; i < wps_number; i++){
      
      if(i < pos.cols()){
         pose_sp.pose.position.x = pos(0, i);
         pose_sp.pose.position.y = pos(1, i);
         pose_sp.pose.position.z = pos(2, i);
      }
      else{
         pose_sp.pose.position.x = old_sp_pos(0);
         pose_sp.pose.position.y = old_sp_pos(1);
         pose_sp.pose.position.z = old_sp_pos(2);
      }

      if(i < yaw.rows()){
         XYZ << 0.0, 0.0, yaw(i);
         quat = utilities::rot2quat( utilities::XYZ2R(XYZ) );
         pose_sp.pose.orientation.w = quat(0);
         pose_sp.pose.orientation.x = quat(1);
         pose_sp.pose.orientation.y = quat(2);
         pose_sp.pose.orientation.z = quat(3);
      }
      else{
         XYZ << 0.0, 0.0, old_sp_yaw;
         quat = utilities::rot2quat( utilities::XYZ2R(XYZ) );
         pose_sp.pose.orientation.w = quat(0);
         pose_sp.pose.orientation.x = quat(1);
         pose_sp.pose.orientation.y = quat(2);
         pose_sp.pose.orientation.z = quat(3);
      }
   
      if( old_sp_pos != pos.block<3,1>(0,i) )
         time_traj_pos = ( pos.block<3,1>(0,i) - old_sp_pos ).norm() / vel;
      else
         time_traj_pos = 0.1;

      if( old_sp_yaw != yaw(i) ){
         if( yaw(i) > old_sp_yaw )
            time_traj_yaw = ( yaw(i) - old_sp_yaw) / vel;
         else if( yaw(i) < _mes_yaw )
            time_traj_yaw = ( old_sp_yaw - yaw(i) ) / vel;
      }
      else
         time_traj_yaw = 0.1;

      if( time_traj_pos >= time_traj_yaw)
         time_traj = time_traj_pos;
      else 
         time_traj = time_traj_yaw;

      if(times.size() > 0)
         time_traj += times[times.size()-1];
               
      times.push_back(time_traj);
      poses.push_back(pose_sp);

      old_sp_yaw = yaw(i);
      old_sp_pos = pos.block<3,1>(0,i);
   }
         
   _trajectory->set_waypoints(poses, times);      
   _trajectory->compute();

}
 
void Navigation::moveToWpsWithYaw( const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> dest, const Eigen::Ref<Eigen::VectorXd> yaw , const double vel ) {

   geometry_msgs::PoseStamped x_traj;
   geometry_msgs::TwistStamped xd_traj;
   geometry_msgs::AccelStamped xdd_traj;
   Eigen::Vector4d att;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d pos_sp;

   double des_vel;

   ros::Rate r(_traj_rate);

   if( vel == 0.0 ) 
      des_vel = _cruise_vel;
   else 
      des_vel = vel;

   //This can work only with the trajectory generator
   if( _act_traj_gen ){

      trajectoryGeneratorWps(dest, yaw, des_vel);

      while( _trajectory->getNext(x_traj, xd_traj, xdd_traj) && _act_traj_gen ){

         pos_sp << x_traj.pose.position.x, x_traj.pose.position.y, x_traj.pose.position.z, 1.0;
         att << x_traj.pose.orientation.w, x_traj.pose.orientation.x, x_traj.pose.orientation.y, x_traj.pose.orientation.z;
         XYZ = utilities::R2XYZ( utilities::QuatToMat(att) );

         _pos_sp = (_H_odom_arena.inverse() * pos_sp).block<3,1>(0,0) ;
         _yaw_sp = XYZ(2);

         r.sleep();
      }
   }

}

void Navigation::moveToWps( const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> dest, const double vel ) {

   geometry_msgs::PoseStamped x_traj;
   geometry_msgs::TwistStamped xd_traj;
   geometry_msgs::AccelStamped xdd_traj;
   Eigen::Vector4d att;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d pos_sp;
   Eigen::VectorXd yaw;

   yaw.resize(1);
   yaw << 0.0;

   double des_vel;

   ros::Rate r(_traj_rate);

   if( vel == 0.0 ) 
      des_vel = _cruise_vel;
   else 
      des_vel = vel;

   //This can work only with the trajectory generator
   if( _act_traj_gen ){

      trajectoryGeneratorWps(dest, yaw, des_vel);

      while( _trajectory->getNext(x_traj, xd_traj, xdd_traj) && _act_traj_gen ){

         pos_sp << x_traj.pose.position.x, x_traj.pose.position.y, x_traj.pose.position.z, 1.0;
         att << x_traj.pose.orientation.w, x_traj.pose.orientation.x, x_traj.pose.orientation.y, x_traj.pose.orientation.z;
         XYZ = utilities::R2XYZ( utilities::QuatToMat(att) );

         _pos_sp = (_H_odom_arena.inverse() * pos_sp).block<3,1>(0,0) ;
         _yaw_sp = XYZ(2);

         r.sleep();
      }
   }

}

void Navigation::moveToWithYaw( const Eigen::Vector3d dest, const double yaw , const double vel){

   geometry_msgs::PoseStamped x_traj;
   geometry_msgs::TwistStamped xd_traj;
   geometry_msgs::AccelStamped xdd_traj;
   Eigen::Vector4d att;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d pos_sp;
   double des_vel;

   ros::Rate r(_traj_rate);

   if( vel == 0.0 ) 
      des_vel = _cruise_vel;
   else  
      des_vel = vel;

   _setpoint = dest;

   if( _act_traj_gen ){

      trajectoryGenerator(dest, yaw, des_vel);

      while( _trajectory->getNext(x_traj, xd_traj, xdd_traj) && _act_traj_gen ){

         pos_sp << x_traj.pose.position.x, x_traj.pose.position.y, x_traj.pose.position.z, 1.0;
         att << x_traj.pose.orientation.w, x_traj.pose.orientation.x, x_traj.pose.orientation.y, x_traj.pose.orientation.z;
         XYZ = utilities::R2XYZ( utilities::QuatToMat(att) );

         _pos_sp = (_H_odom_arena.inverse() * pos_sp).block<3,1>(0,0) ;
         _yaw_sp = XYZ(2);

         r.sleep();
      }
   }
   else{
      
      pos_sp << dest(0), dest(1), dest(2), 1.0;
      _pos_sp = (_H_odom_arena.inverse() * pos_sp).block<3,1>(0,0);
      _yaw_sp = yaw;

   }   

}

void Navigation::moveTo( const Eigen::Vector3d dest, const double vel){

   geometry_msgs::PoseStamped x_traj;
   geometry_msgs::TwistStamped xd_traj;
   geometry_msgs::AccelStamped xdd_traj;
   Eigen::Vector4d att;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d pos_sp;
   double yaw = _mes_yaw;
   double des_vel;

   ros::Rate r(_traj_rate);

   if( vel == 0.0 ) 
      des_vel = _cruise_vel;
   else  
      des_vel = vel;

   _setpoint = dest;

   cout << "sp_arena: " << dest.transpose() << endl;
   Eigen::Vector4d destt;
   destt << dest(0), dest(1), dest(2), 1.0;
   cout << "sp_odom: " << (_H_odom_arena.inverse() * destt).transpose() << endl;

   if( _act_traj_gen ){

      trajectoryGenerator(dest, yaw, des_vel);

      while( _trajectory->getNext(x_traj, xd_traj, xdd_traj) && _act_traj_gen ){
         pos_sp << x_traj.pose.position.x, x_traj.pose.position.y, x_traj.pose.position.z, 1.0;
         att << x_traj.pose.orientation.w, x_traj.pose.orientation.x, x_traj.pose.orientation.y, x_traj.pose.orientation.z;
         XYZ = utilities::R2XYZ( utilities::QuatToMat(att) );
         
         _pos_sp = (_H_odom_arena.inverse() * pos_sp ).block<3,1>(0,0);
         _yaw_sp = XYZ(2);
         
         //cout << "sp_map: " << pos_sp.transpose() << endl;
         //cout << "sp_odo: " << _pos_sp.transpose() << endl;

         r.sleep();
      }
   }
   else{

      pos_sp << dest(0), dest(1), dest(2), 1.0;
      _pos_sp = (_H_odom_arena.inverse() * pos_sp).block<3,1>(0,0);
      _yaw_sp = yaw;

   } 

}

void Navigation::land( double altitude, double vel) {  

   Eigen::Vector3d pos_sp;
   double des_vel;
   
   pos_sp << _world_pos(0), _world_pos(1), altitude + _height_threshold;

   if(vel == 0.0)
      des_vel = _cruise_vel;
   else 
      des_vel = vel;
   
   moveTo(pos_sp, vel);

   //cout << "Land px4 \n";
   mavros_msgs::CommandTOL land_srv;
   _land_client.call( land_srv );
   while( _mstate.armed ) usleep(0.1*1e6);
   cout << "Disarmed!" << endl;
   _take_off = false;
   
}

void Navigation::setWorldTransform(const Eigen::Ref<Eigen::Matrix<double, 4, 4>> new_H_odom_Arena){ 
   
   _H_odom_arena = new_H_odom_Arena;
}


void Navigation::tf_broadcast_poses(){

   tf::Transform transform_mb;
   tf::Transform transform_mo;
   tf::Transform transform_ob;

   ros::Rate r(5);
   while (ros::ok() ) {     
      transform_mb.setOrigin(tf::Vector3(_world_pos(0),_world_pos(1),_world_pos(2)));
      tf::Quaternion q_mb(_world_quat(1),_world_quat(2),_world_quat(3),_world_quat(0));
      transform_mb.setRotation(q_mb);
      tf::StampedTransform stamp_transform_mb(transform_mb, ros::Time::now(), "map_vis", "base_link_vis");



      transform_mo.setOrigin(tf::Vector3(_H_odom_arena(0,3),_H_odom_arena(1,3),_H_odom_arena(2,3)));
      Eigen::Vector4d quat_mo;
      quat_mo = utilities::rot2quat( _H_odom_arena.block<3,3>(0,0) );
      tf::Quaternion q_mo( quat_mo(1),quat_mo(2),quat_mo(3),quat_mo(0) );
      transform_mo.setRotation(q_mo);
      tf::StampedTransform stamp_transform_mo(transform_mo, ros::Time::now(), "map_vis", "odom_vis");
      
      transform_ob.setOrigin(tf::Vector3(_world_pos_odom(0),_world_pos_odom(1),_world_pos_odom(2)));
      tf::Quaternion q_ob(_world_quat_odom(1),_world_quat_odom(2),_world_quat_odom(3),_world_quat_odom(0));
      transform_ob.setRotation(q_ob);
      tf::StampedTransform stamp_transform_ob(transform_ob, ros::Time::now(), "odom_vis", "base_link_vis");
      
      _broadcaster.sendTransform(stamp_transform_mo);
      _broadcaster.sendTransform(stamp_transform_mb);
      _broadcaster.sendTransform(stamp_transform_ob);


    }

}
 

void Navigation::tf_broadcast_pose_arena(){

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(_world_pos(0),_world_pos(1),_world_pos(2)));
    tf::Quaternion q(_world_quat(1),_world_quat(2),_world_quat(3),_world_quat(0));
    transform.setRotation(q);
    tf::StampedTransform stamp_transform(transform, ros::Time::now(), "map", "base_link");
    _broadcaster.sendTransform(stamp_transform);

}
 
void Navigation::tf_broadcast_odom_arena(){

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(_H_odom_arena(0,3),_H_odom_arena(1,3),_H_odom_arena(2,3)));
    Eigen::Vector4d quat;
    quat = utilities::rot2quat( _H_odom_arena.block<3,3>(0,0) );
    tf::Quaternion q( quat(1),quat(2),quat(3),quat(0) );
    transform.setRotation(q);
    tf::StampedTransform stamp_transform(transform, ros::Time::now(), "map", "odom");
    _broadcaster.sendTransform(stamp_transform);

}

void Navigation::tf_broadcast_pose_odom(){

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(_world_pos_odom(0),_world_pos_odom(1),_world_pos_odom(2)));
    tf::Quaternion q(_world_quat_odom(1),_world_quat_odom(2),_world_quat_odom(3),_world_quat_odom(0));
    transform.setRotation(q);
    tf::StampedTransform stamp_transform(transform, ros::Time::now(), "odom", "base_link");
    _broadcaster.sendTransform(stamp_transform);

}



