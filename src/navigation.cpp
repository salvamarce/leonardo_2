#include "navigation.h"

int sign(double p){
   if ( p < 0.0)
      return -1;
   else
      return 1;
}

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
      _cruise_vel = 0.4;
   }

   if( !_nh.getParam("max_cruise_acc", _cruise_acc)) {
      _cruise_acc = 1.2;
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

   // --- Publisher and Subscribers ---
   _setpoint_pub = _nh.advertise<mavros_msgs::PositionTarget>( _setpoint_topic.c_str() , 1);
   _setpoint_bag_pub = _nh.advertise<geometry_msgs::Point>( "setpoint_bag" , 1);
   _pose_sub = _nh.subscribe( _pose_topic.c_str() , 1, &Navigation::pose_cb, this);
   _mavros_state_sub = _nh.subscribe( _mavros_state_topic.c_str(), 1, &Navigation::mavros_state_cb, this);

   // --- Services ---
   _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
   _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
   _land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
   //---

   // --- Trajectory ---
   _first_local_pos = false;
   _trajectory = new CARTESIAN_PLANNER(_traj_rate);
   _take_off = false;
   _act_traj_gen = true;
   _interrupt = false;
   _localization_status = false;

   // --- Drone States ---
   _H_odom_arena = Eigen::Matrix4d::Identity();
   _world_pos << 0.0, 0.0, _drone_height, 1.0;
   _world_pos_odom << 0.0, 0.0, 0.0, 1.0;
   _world_quat << 1.0, 0.0, 0.0, 0.0;
   _world_quat_odom << 1.0, 0.0, 0.0, 0.0;
   _old_setpoint << 0.0, 0.0, 0.0;

   // --- TOF variables ---
   if( !_nh.getParam("critical_distance", _critical_distance)) {
      _critical_distance = 400.0;
   }
   _critical_state = false;
   load_tof_angles();

}

void Navigation::load_tof_angles(){

   string tof;

   for(int i=0; i<8; i++){
      tof = "angle_tof_" + to_string(i);
      
      _nh.getParam(tof, _tof_angles(i));

   }

}

void Navigation::mavros_state_cb( mavros_msgs::State mstate) {
   _mstate = mstate;
}

void Navigation::pose_cb ( geometry_msgs::PoseStamped msg ) {

   Eigen::Vector4d temp;
   Eigen::Vector3d rpy;

   _world_pos_odom << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1.0;

   _world_pos = _H_odom_arena * _world_pos_odom;
   //_world_pos(2) += _drone_height;

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

   while( !arrived( pos_sp )) {
      sleep(0.001);
   }

   _take_off = true;

   ROS_INFO("Takeoff completed");
   

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
      //cout << "e_pos: " << epx << " " << epy << " " << epz << endl;
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
   geometry_msgs::Point bag_target;
   
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

   des_pos  = _world_pos_odom.block<3,1>(0,0);
   des_yaw = 0.0;

   _des_pos_sp = des_pos;
   _des_yaw_sp = des_yaw;

   while( ros::ok() ){

      if( _mstate.mode != "OFFBOARD") {
         
         des_pos  = _world_pos_odom.block<3,1>(0,0);
         des_yaw = 0.0;
      }
      else if( isnan( _pos_sp(0) ) || isnan( _pos_sp(1) ) || isnan( _pos_sp(2) ) || isnan( _yaw_sp ) ){

         cout << "pos_sp is NaN " << _pos_sp.transpose() << " yaw: " << _yaw_sp << endl;
         des_pos  = _world_pos_odom.block<3,1>(0,0);
         des_yaw = 0.0;

      }
      else{

         
         //des_pos = _H_odom_arena.block<3,3>(0,0) * _pos_sp;
         des_pos = _pos_sp;

         //if(des_pos(2) < 0.0) 
           // des_pos(2) = 0.0;
         //else 
         if(des_pos(2) > _max_height)
            des_pos(2) = _max_height;

         des_yaw = _yaw_sp;

      }
      
      _des_pos_sp = des_pos;
      _des_yaw_sp = des_yaw;
      ptarget.header.stamp = ros::Time::now();
      ptarget.position.x = des_pos(0);
      ptarget.position.y = des_pos(1);
      ptarget.position.z = des_pos(2);  
      bag_target.x = des_pos(0);
      bag_target.y = des_pos(1);
      bag_target.z = des_pos(2);
      // ptarget.velocity.y = _vel_sp(1);
      // ptarget.velocity.x = _vel_sp(0);
      // ptarget.velocity.z = _vel_sp(2);        
      ptarget.yaw = des_yaw ;
      
      _setpoint_pub.publish( ptarget );
      _setpoint_bag_pub.publish(bag_target);
     
      r.sleep();
   }

}

void Navigation::trajectoryGenerator(const Eigen::Vector3d pos, const double yaw, const double vel){

   std::vector<double> times;
   std::vector<geometry_msgs::PoseStamped> poses;
   geometry_msgs::PoseStamped pose_sp;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d quat;
   double time_traj_pos, time_traj_pos_x, time_traj_pos_y, time_traj_pos_z, time_traj_yaw;

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
      time_traj_pos_x = fabs( pos(0) - _world_pos(0) ) / vel;
      time_traj_pos_y = fabs( pos(1) - _world_pos(1) ) / vel;
      time_traj_pos_z = fabs( pos(2) - _world_pos(2) ) / vel;

      if( time_traj_pos_x >= time_traj_pos_y && time_traj_pos_x >= time_traj_pos_z )
         time_traj_pos = time_traj_pos_x;
      else if( time_traj_pos_y >= time_traj_pos_x && time_traj_pos_y >= time_traj_pos_z )
         time_traj_pos = time_traj_pos_y;
      else if( time_traj_pos_z >= time_traj_pos_x && time_traj_pos_z >= time_traj_pos_y )
         time_traj_pos = time_traj_pos_z;
   }
   else
      time_traj_pos = 0.1;

   cout << "time_x: " << time_traj_pos_x << endl;
   cout << "time_y: " << time_traj_pos_y << endl;
   cout << "time_z: " << time_traj_pos_z << endl;
   cout << "time : " << time_traj_pos << endl;
   

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
   double time_traj_pos, time_traj_pos_x, time_traj_pos_y, time_traj_pos_z, time_traj_yaw;
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

      if( old_sp_pos != pos.block<3,1>(0,i) ){
         time_traj_pos_x = fabs( pos(0, i) - old_sp_pos(0) ) / vel;
         time_traj_pos_y = fabs( pos(1, i) - old_sp_pos(1) ) / vel;
         time_traj_pos_z = fabs( pos(2, i) - old_sp_pos(2) ) / vel;

         if( time_traj_pos_x >= time_traj_pos_y && time_traj_pos_x >= time_traj_pos_z )
            time_traj_pos = time_traj_pos_x;
         else if( time_traj_pos_y >= time_traj_pos_x && time_traj_pos_y >= time_traj_pos_z )
            time_traj_pos = time_traj_pos_y;
         else if( time_traj_pos_z >= time_traj_pos_x && time_traj_pos_z >= time_traj_pos_y )
            time_traj_pos = time_traj_pos_z;
      }
      else
         time_traj_pos = 0.1;


      cout << "time_x: " << time_traj_pos_x << endl;
      cout << "time_y: " << time_traj_pos_y << endl;
      cout << "time_z: " << time_traj_pos_z << endl;
      cout << "time : " << time_traj_pos << endl;

      if(i < yaw.rows()){
         if( old_sp_yaw != yaw(i) ){
            if( yaw(i) > old_sp_yaw )
               time_traj_yaw = ( yaw(i) - old_sp_yaw) / vel;
            else if( yaw(i) < _mes_yaw )
               time_traj_yaw = ( old_sp_yaw - yaw(i) ) / vel;
         }
         else
            time_traj_yaw = 0.1;
      }
      else
         time_traj_yaw = 0.1;

      if( time_traj_pos >= time_traj_yaw)
         time_traj = time_traj_pos;
      else 
         time_traj = time_traj_yaw;

      if(times.size() > 0)
         time_traj += times[times.size()-1];

      cout << "time_traj: " << time_traj << endl;
               
      times.push_back(time_traj);

      poses.push_back(pose_sp);
     
      
      if(i < yaw.rows()){
         old_sp_yaw = yaw(i);
      }
      else
         old_sp_yaw = old_sp_yaw;

      if(i < pos.cols()){
         old_sp_pos = pos.block<3,1>(0,i);
      }
      else
         old_sp_pos = old_sp_pos;
      
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
   Eigen::Matrix4d sp;
   sp = Eigen::Matrix4d::Identity();

   double des_vel;

   ros::Rate r(_traj_rate);

   if( vel == 0.0 ) 
      des_vel = _cruise_vel;
   else 
      des_vel = vel;

   //This can work only with the trajectory generator
   if( _act_traj_gen ){

      trajectoryGeneratorWps(dest, yaw, des_vel);
      
      while( _trajectory->getNext(x_traj, xd_traj, xdd_traj) && _act_traj_gen && !_interrupt ){

         pos_sp << x_traj.pose.position.x, x_traj.pose.position.y, x_traj.pose.position.z, 1.0;
         _vel_sp << xd_traj.twist.linear.x, xd_traj.twist.linear.y, xd_traj.twist.linear.z;
         att << x_traj.pose.orientation.w, x_traj.pose.orientation.x, x_traj.pose.orientation.y, x_traj.pose.orientation.z;
         
         sp.block<3,3>(0,0) = utilities::QuatToMat(att);
         sp.block<4,1>(0,3) = pos_sp;

         sp = _H_odom_arena.inverse() * sp;

         _pos_sp = sp.block<3,1>(0,3);
         _yaw_sp = utilities::R2XYZ( sp.block<3,3>(0,0))(2);

         r.sleep();
      }

      if( _interrupt ){
         _pos_sp = _old_setpoint;
         _yaw_sp = 0.0;
      }

   }

}

void Navigation::moveToWps( const Eigen::Ref<Eigen::Matrix<double, 3, Dynamic>> dest, const double vel ) {
   /*
   Eigen::Vector4d att;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d pos_sp;
   Eigen::Matrix4d sp;
   double des_vel;
   double time_traj_pos_x, time_traj_pos_y, time_traj_pos_z;
   double dt, act_time;
   double tc_plus;
   Eigen::Matrix<double, 3, Dynamic>  qd_c, qdd_c, tc, qi, qf;
   Eigen::Matrix<double, Dynamic, 1> tf, overlap;
   double qdd_c_traj;

   int col = dest.cols() - 1;

   ros::Rate r(_traj_rate);

   qd_c.resize(3,col);
   qdd_c.resize(3,col);
   tc.resize(3,col);
   qi.resize(3,col);
   qf.resize(3,col);
   tf.resize(col,1);
   overlap.resize(col-1, 1);

   dt = 1.0/_traj_rate;

   if( vel == 0.0 ) 
      des_vel = _cruise_vel;
   else  
      des_vel = vel;

   tc_plus = des_vel / _cruise_acc;

   
   // qi.block<3,1>(0,0) = _world_pos.block<3,1>(0,0);
   // qf.block<3,1>(0,0) = dest.block<3,1>(0,1);

   // time_traj_pos_x = fabs( qf(0) - qi(0) ) / des_vel + 2.0 * tc_plus;
   // time_traj_pos_y = fabs( qf(1) - qi(1) ) / des_vel + 2.0 * tc_plus;
   // time_traj_pos_z = fabs( qf(2) - qi(2) ) / des_vel + 2.0 * tc_plus;

   // if( time_traj_pos_x >= time_traj_pos_y && time_traj_pos_x >= time_traj_pos_z )
   //    tf(0) = time_traj_pos_x;
   // else if( time_traj_pos_y >= time_traj_pos_x && time_traj_pos_y >= time_traj_pos_z )
   //    tf(0) = time_traj_pos_y;
   // else if( time_traj_pos_z >= time_traj_pos_x && time_traj_pos_z >= time_traj_pos_y )
   //    tf(0) = time_traj_pos_z;

   // qd_c(0,0) = sign( qf(0) - qi(0) ) * des_vel;
   // qd_c(1,0) = sign( qf(1) - qi(1) ) * des_vel;
   // qd_c(2,0) = sign( qf(2) - qi(2) ) * des_vel;

   // qdd_c(0,0) = pow( qd_c(0,0), 2) / ( qi(0) - qf(0) + qd_c(0,0) * tf(0) );
   // qdd_c(1,0) = pow( qd_c(1,0), 2) / ( qi(1) - qf(1) + qd_c(1,0) * tf(0) );
   // qdd_c(2,0) = pow( qd_c(2,0), 2) / ( qi(2) - qf(2) + qd_c(2,0) * tf(0) );

   // tc(0,0) = 0.5 * tf(0) - 0.5 * sqrt( (pow(tf(0),2) * qdd_c(0,0) - 4*(qf(0)-qi(0))) / qdd_c(0,0) );
   // tc(1,0) = 0.5 * tf(0) - 0.5 * sqrt( (pow(tf(0),2) * qdd_c(1,0) - 4*(qf(1)-qi(1))) / qdd_c(1,0) );
   // tc(2,0) = 0.5 * tf(0) - 0.5 * sqrt( (pow(tf(0),2) * qdd_c(2,0) - 4*(qf(2)-qi(2))) / qdd_c(2,0) );
   

   for( int i=0; i<col; i++ ){

      if( i== 0 )
         qi.block<3,1>(0,i) = _world_pos.block<3,1>(0,0);
      else
         qi.block<3,1>(0,i) = dest.block<3,1>(0,i);

      qf.block<3,1>(0,i) = dest.block<3,1>(0,i+1);

      time_traj_pos_x = fabs( qf(0,i) - qi(0,i) ) / des_vel + 2.0 * tc_plus;
      time_traj_pos_y = fabs( qf(1,i) - qi(1,i) ) / des_vel + 2.0 * tc_plus;
      time_traj_pos_z = fabs( qf(2,i) - qi(2,i) ) / des_vel + 2.0 * tc_plus;

      if( time_traj_pos_x >= time_traj_pos_y && time_traj_pos_x >= time_traj_pos_z )
         tf(i) = time_traj_pos_x;
      else if( time_traj_pos_y >= time_traj_pos_x && time_traj_pos_y >= time_traj_pos_z )
         tf(i) = time_traj_pos_y;
      else if( time_traj_pos_z >= time_traj_pos_x && time_traj_pos_z >= time_traj_pos_y )
         tf(i) = time_traj_pos_z;

      qd_c(0,i) = sign( qf(0,i) - qi(0,i) ) * des_vel;
      qd_c(1,i) = sign( qf(1,i) - qi(1,i) ) * des_vel;
      qd_c(2,i) = sign( qf(2,i) - qi(2,i) ) * des_vel;

      qdd_c(0,i) = pow( qd_c(0,i), 2) / ( qi(0,i) - qf(0,i) + qd_c(0,i) * tf(i) );
      qdd_c(1,i) = pow( qd_c(1,i), 2) / ( qi(1,i) - qf(1,i) + qd_c(1,i) * tf(i) );
      qdd_c(2,i) = pow( qd_c(2,i), 2) / ( qi(2,i) - qf(2,i) + qd_c(2,i) * tf(i) );

      tc(0,i) = 0.5 * tf(i) - 0.5 * sqrt( (pow(tf(i),2) * qdd_c(0,i) - 4*(qf(0,i)-qi(0,i))) / qdd_c(0,i) );
      tc(1,i) = 0.5 * tf(i) - 0.5 * sqrt( (pow(tf(i),2) * qdd_c(1,i) - 4*(qf(1,i)-qi(1,i))) / qdd_c(1,i) );
      tc(2,i) = 0.5 * tf(i) - 0.5 * sqrt( (pow(tf(i),2) * qdd_c(2,i) - 4*(qf(2,i)-qi(2,i))) / qdd_c(2,i) );

   }
   cout << "qi: " << qi << endl;
   cout << "qf: " << qf << endl;
   cout << "qd_c: " << qd_c << endl;
   cout << "qdd_c: " << qdd_c << endl;
   cout << "tc: " << tc << endl;
   cout << "tf: " << tf.transpose() << endl;
   
   int i=0;

   for(int i=0; i<col-1; i++){

      if( tf(i) <= tf(i+1) && tf(i) <= 1.0 ){
         cout << "if1 \n";
         overlap(i) = tf(i);
      }
      else if( tf(i+1) <= tf(i) && tf(i+1) <= 1.0 ){
         cout << "if2 \n";
         overlap(i) = tf(i+1);
      }
      else{
         cout << "if3 \n";
         overlap(i) = 1.0;
      }

      overlap(i) = 0.0;
      cout << "overlap( "<< i << ") " << overlap(i) << endl;

   }

   while( i < col && !_interrupt ){
      cout << "ciclo # " << i << endl;
      cout << "col: " << col << endl;

      act_time = 0.0;

      while( act_time <= tf(i) && !_interrupt ){

         cout << "tf(i): " << tf(i) << endl;
         cout << "act_time: " << act_time << endl;

         for(int j=0; j<3; j++){
            
            if( i < col-1 && i > 0 ){
               if( act_time >= (tf(i) - overlap(i-1) ) ){
                  cout << "over con +1 \n";
                  qdd_c_traj = qdd_c(j,i) + qdd_c(j,i+1);
               }
               else if( act_time <= (tf(i-1) + overlap(i-1) ) ){
                  cout << "over con -1 \n";
                  qdd_c_traj = qdd_c(j,i) + qdd_c(j,i-1);
               }
               else
                  qdd_c_traj = qdd_c(j,i);
            }
            else if( i == 0 ){
               if( act_time >= (tf(i) - overlap(0) ) ){
                  cout << "over con +1 \n";
                  qdd_c_traj = qdd_c(j,i) + qdd_c(j,i+1);
               }
               else
                  qdd_c_traj = qdd_c(j,i);
            }
            else if( i == col-1 ){
               if( act_time <= (tf(i-1) + overlap(i-1) ) ){
                  cout << "over con -1 \n";
                  qdd_c_traj = qdd_c(j,i) + qdd_c(j,i-1);
               }
               else
                  qdd_c_traj = qdd_c(j,i);
            }

            // cout << "qdd_c_traj: " << qdd_c_traj << endl;

            if( act_time <= tc(j,i) ){
               cout << "accellero \n";
               pos_sp(j) = qi(j,i) + 0.5 * qdd_c_traj * pow(act_time, 2);
            }            
            else if( act_time > tc(j,i) && act_time <= (tf(i)-tc(j,i)) ){
               cout << "crociera \n" << endl;
               pos_sp(j) = qi(j,i) + qdd_c_traj * tc(j,i) * (act_time - 0.5*tc(j,i));
            }            
            else if( act_time > (tf(i)-tc(j)) && act_time <= tf(i) ){
               cout << "decellero \n";
               pos_sp(j) = qf(j,i) - 0.5 * qdd_c_traj * pow( (tf(i)-act_time), 2);
            }

         }

         sp.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
         sp.block<3,1>(0,3) = pos_sp.block<3,1>(0,0);
         sp(3,3) = 1.0;

         sp = _H_odom_arena.inverse() * sp;

         _pos_sp = sp.block<3,1>(0,3);
         _yaw_sp = utilities::R2XYZ( sp.block<3,3>(0,0))(2);
         
         cout << "sp_map: " << pos_sp.transpose() << endl;
         cout << "qdd_c: " << qdd_c_traj << endl;
         // cout << "sp_odo: " << _pos_sp.transpose() << endl;
         //cout << "act_time: " << act_time << endl;
         act_time += dt;
         r.sleep();
      }

      i++;
   }

   if( _interrupt ){
      
      _pos_sp = _world_pos_odom.block<3,1>(0,0);
      _yaw_sp = 0.0;

      _interrupt = false;
   }
   
   */

   for(int i=0; i<dest.cols(); i++){
      moveTo( dest.block<3,1>(0,i) );
      sleep(0.001);
      if( _interrupt)
         break;
   }
   
}

void Navigation::moveToWithYaw( const Eigen::Vector3d dest, const double yaw , const double vel){

   geometry_msgs::PoseStamped x_traj;
   geometry_msgs::TwistStamped xd_traj;
   geometry_msgs::AccelStamped xdd_traj;
   Eigen::Vector4d att;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d pos_sp;
   Eigen::Matrix4d sp;
   sp = Eigen::Matrix4d::Identity();
   double des_vel;

   ros::Rate r(_traj_rate);

   if( vel == 0.0 ) 
      des_vel = _cruise_vel;
   else  
      des_vel = vel;

   _setpoint = dest;

   if( _act_traj_gen ){

      trajectoryGenerator(dest, yaw, des_vel);

      while( _trajectory->getNext(x_traj, xd_traj, xdd_traj) && _act_traj_gen && !_interrupt ){

         pos_sp << x_traj.pose.position.x, x_traj.pose.position.y, x_traj.pose.position.z, 1.0;
         att << x_traj.pose.orientation.w, x_traj.pose.orientation.x, x_traj.pose.orientation.y, x_traj.pose.orientation.z;

         sp.block<3,3>(0,0) = utilities::QuatToMat(att);
         sp.block<4,1>(0,3) = pos_sp;

         sp = _H_odom_arena.inverse() * sp;

         _pos_sp = sp.block<3,1>(0,3);
         _yaw_sp = utilities::R2XYZ( sp.block<3,3>(0,0))(2);

         r.sleep();
      }
      
      if( _interrupt ){
         _pos_sp = _world_pos_odom.block<3,1>(0,0);
         _yaw_sp = 0.0;
      }
   }
   else{
      
      //Terna odom
      if( !_interrupt ){
         _pos_sp = dest;
         _yaw_sp = yaw;
      }
      else{
         _pos_sp = _world_pos_odom.block<3,1>(0,0);
         _yaw_sp = 0.0;
      }

   }   

}

/* -- moveTo con SPLINE 
void Navigation::moveTo( const Eigen::Vector3d dest, const double vel){

   geometry_msgs::PoseStamped x_traj;
   geometry_msgs::TwistStamped xd_traj;
   geometry_msgs::AccelStamped xdd_traj;
   Eigen::Vector4d att;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d pos_sp;
   Eigen::Matrix4d sp;
   double yaw ;
   double des_vel;

   ros::Rate r(_traj_rate);

   if( vel == 0.0 ) 
      des_vel = _cruise_vel;
   else  
      des_vel = vel;

   _setpoint = dest;

   yaw = 0.0;

   cout << "sp_arena: " << dest.transpose() << endl;
   Eigen::Vector4d destt;
   destt << dest(0), dest(1), dest(2), 1.0;
   cout << "sp_odom: " << (_H_odom_arena.inverse() * destt).transpose() << endl;

   sp = Eigen::Matrix4d::Identity();

   if( _act_traj_gen ){

      trajectoryGenerator(dest, yaw, des_vel);

      while( _trajectory->getNext(x_traj, xd_traj, xdd_traj) && _act_traj_gen && !_interrupt ){
         pos_sp << x_traj.pose.position.x, x_traj.pose.position.y, x_traj.pose.position.z, 1.0;
         _vel_sp << xd_traj.twist.linear.x, xd_traj.twist.linear.y, xd_traj.twist.linear.z;
         att << x_traj.pose.orientation.w, x_traj.pose.orientation.x, x_traj.pose.orientation.y, x_traj.pose.orientation.z;
         
         sp.block<3,3>(0,0) = utilities::QuatToMat(att);
         sp.block<4,1>(0,3) = pos_sp;

         sp = _H_odom_arena.inverse() * sp;

         _pos_sp = sp.block<3,1>(0,3);
         _yaw_sp = utilities::R2XYZ( sp.block<3,3>(0,0))(2);
         
         // cout << "sp_map: " << pos_sp.transpose() << endl;
         // cout << "sp_odo: " << _pos_sp.transpose() << endl;

         r.sleep();
      }

      if( _interrupt ){
         
         _pos_sp = _world_pos_odom.block<3,1>(0,0);
         _yaw_sp = 0.0;

         _interrupt = false;
      }
   }
   else{

      //Terna odom
      if ( !_interrupt ){
         _pos_sp = dest;
         _yaw_sp = 0.0;
      }
      else{
         _pos_sp = _world_pos_odom.block<3,1>(0,0);
         _yaw_sp = 0.0;

         _interrupt = false;

      }

   } 

}
*/

// --- moveTo con trapezoidale ---
void Navigation::moveTo( const Eigen::Vector3d dest, const double vel){

   Eigen::Vector4d att;
   Eigen::Vector3d XYZ;
   Eigen::Vector4d pos_sp;
   Eigen::Matrix4d sp;
   double des_vel;
   Eigen::Vector3d qd_c, qdd_c, tc, qi, qf;
   double tf, time_traj_pos_x, time_traj_pos_y, time_traj_pos_z;
   double dt, act_time;
   double tc_plus;

   ros::Rate r(_traj_rate);

   dt = 1.0/_traj_rate;

   if( vel == 0.0 ) 
      des_vel = _cruise_vel;
   else  
      des_vel = vel;

   _setpoint = dest;

   sp = Eigen::Matrix4d::Identity();

   tc_plus = des_vel / _cruise_acc;

   // cout << "tc_plus: " << tc_plus << endl;

   if( _act_traj_gen ){

      cout << "sp_arena: " << dest.transpose() << endl;
      Eigen::Vector4d destt;
      destt << dest(0), dest(1), dest(2), 1.0;
      cout << "sp_odom: " << (_H_odom_arena.inverse() * destt).transpose() << endl;

      qi = _world_pos.block<3,1>(0,0);
      qf = dest;

      time_traj_pos_x = fabs( qf(0) - qi(0) ) / des_vel + 2.0 * tc_plus;
      time_traj_pos_y = fabs( qf(1) - qi(1) ) / des_vel + 2.0 * tc_plus;
      time_traj_pos_z = fabs( qf(2) - qi(2) ) / des_vel + 2.0 * tc_plus;

      if( time_traj_pos_x >= time_traj_pos_y && time_traj_pos_x >= time_traj_pos_z )
         tf = time_traj_pos_x;
      else if( time_traj_pos_y >= time_traj_pos_x && time_traj_pos_y >= time_traj_pos_z )
         tf = time_traj_pos_y;
      else if( time_traj_pos_z >= time_traj_pos_x && time_traj_pos_z >= time_traj_pos_y )
         tf = time_traj_pos_z;

      // cout << "tf: " << tf << endl;

      qd_c(0) = sign( qf(0) - qi(0) ) * des_vel;
      qd_c(1) = sign( qf(1) - qi(1) ) * des_vel;
      qd_c(2) = sign( qf(2) - qi(2) ) * des_vel;

      // cout << "qd_c: " << qd_c.transpose() << endl;

      qdd_c(0) = pow( qd_c(0), 2) / ( qi(0) - qf(0) + qd_c(0) * tf );
      qdd_c(1) = pow( qd_c(1), 2) / ( qi(1) - qf(1) + qd_c(1) * tf );
      qdd_c(2) = pow( qd_c(2), 2) / ( qi(2) - qf(2) + qd_c(2) * tf );

      // cout << "qdd_c: " << qdd_c.transpose() << endl;

      tc(0) = 0.5 * tf - 0.5 * sqrt( (pow(tf,2) * qdd_c(0) - 4*(qf(0)-qi(0))) / qdd_c(0) );
      tc(1) = 0.5 * tf - 0.5 * sqrt( (pow(tf,2) * qdd_c(1) - 4*(qf(1)-qi(1))) / qdd_c(1) );
      tc(2) = 0.5 * tf - 0.5 * sqrt( (pow(tf,2) * qdd_c(2) - 4*(qf(2)-qi(2))) / qdd_c(2) );
      
      // cout << "tc: " << tc.transpose() << endl;

      act_time = 0.0;

      while( act_time <= tf && !_interrupt ){
         cout << "move_to, con traj. inter:" << _interrupt << endl;

         for(int j=0; j<3; j++){

            if( act_time <= tc(j) )
               pos_sp(j) = qi(j) + 0.5 * qdd_c(j) * pow(act_time, 2);
            
            else if( act_time > tc(j) && act_time <= (tf-tc(j)) )
               pos_sp(j) = qi(j) + qdd_c(j) * tc(j) * (act_time - 0.5*tc(j));
            
            else if( act_time > (tf-tc(j)) && act_time <= tf )
               pos_sp(j) = qf(j) - 0.5 * qdd_c(j) * pow( (tf-act_time), 2);

         }

         // cout << "pos_sp: " << pos_sp.transpose() << endl;
         sp.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
         sp.block<3,1>(0,3) = pos_sp.block<3,1>(0,0);

         sp = _H_odom_arena.inverse() * sp;

         _pos_sp = sp.block<3,1>(0,3);
         _yaw_sp = utilities::R2XYZ( sp.block<3,3>(0,0))(2);
         
         // cout << "sp_map: " << pos_sp.transpose() << endl;
         // cout << "sp_odo: " << _pos_sp.transpose() << endl;
         //cout << "act_time: " << act_time << endl;
         act_time += dt;
         r.sleep();
      }

      if( _interrupt ){
         
         _pos_sp = _world_pos_odom.block<3,1>(0,0);
         _yaw_sp = 0.0;
      }
   }
   else{

         cout << "sp_odom: " << dest.transpose() << endl;
         
      //Terna odom
      if ( !_interrupt ){
         _pos_sp = dest;
         _yaw_sp = 0.0;
      }
      else{
         _pos_sp = _world_pos_odom.block<3,1>(0,0);
         _yaw_sp = 0.0;

      }

   } 

}

void Navigation::land( double altitude, double vel) {  

   Eigen::Vector3d pos_sp;
   double des_vel;
   
   pos_sp << _world_pos(0), _world_pos(1), altitude;

   if(vel == 0.0)
      des_vel = _cruise_vel;
   else 
      des_vel = vel;
   
   _act_traj_gen = true;
   
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

   _world_pos = _H_odom_arena * _world_pos_odom;
   cout << "corretto !! \n";

   _localization_status = true;
}

void Navigation::tof_reads_cb( std_msgs::Float32MultiArray values ){

   double min = _critical_distance;
   int tof = 0;
   bool critical = false;
   
   for(int i=0; i<8; i++){
      if(values.data[i] <= _critical_distance){
         critical = true;

         if( values.data[i] <= min ){
            min = values.data[i];
            tof = i;         
         }
      }
   }

   if(critical){
      _critical_state = true; 
      cout << "Distanza critica \n";
   }  

}


// --- TF functions ---

void Navigation::tf_broadcast_poses(){

   int comm_sp;
   create_socket("192.168.0.99", 9090, &comm_sp);


   ros::Rate r(5);

   Poses3D tf_poses;

   while(ros::ok()) {

      tf_poses.bl_a.x = _world_pos(0);
      tf_poses.bl_a.y = _world_pos(1); 
      tf_poses.bl_a.z = _world_pos(2);

      tf_poses.bl_a.qw = _world_quat(0);
      tf_poses.bl_a.qx = _world_quat(1);
      tf_poses.bl_a.qy = _world_quat(2);
      tf_poses.bl_a.qz = _world_quat(3);

      tf_poses.o_a.x = _H_odom_arena(0,3);
      tf_poses.o_a.y = _H_odom_arena(1,3); 
      tf_poses.o_a.z = _H_odom_arena(2,3);
      Eigen::Vector4d quat_o_a;
      quat_o_a = utilities::rot2quat( _H_odom_arena.block<3,3>(0,0) );

      tf_poses.o_a.qw = quat_o_a(0);
      tf_poses.o_a.qx = quat_o_a(1);
      tf_poses.o_a.qy = quat_o_a(2);
      tf_poses.o_a.qz = quat_o_a(3);

      tf_poses.bl_o.x = _world_pos_odom(0);
      tf_poses.bl_o.y = _world_pos_odom(1); 
      tf_poses.bl_o.z = _world_pos_odom(2);

      tf_poses.bl_o.qw = _world_quat_odom(0);
      tf_poses.bl_o.qx = _world_quat_odom(1);
      tf_poses.bl_o.qy = _world_quat_odom(2);
      tf_poses.bl_o.qz = _world_quat_odom(3);

      int n = write( comm_sp, &tf_poses, sizeof(tf_poses));
      //cout << "Scritti: " << n << endl;

      r.sleep();
   }

   /*
   tf::Transform transform_mb;
   tf::Transform transform_mo;
   tf::Transform transform_ob;

   ros::Rate r(5);
   while (ros::ok() ) {     
      transform_mb.setOrigin(tf::Vector3(_world_pos(0),_world_pos(1),_world_pos(2)));
      tf::Quaternion q_mb(_world_quat(1),_world_quat(2),_world_quat(3),_world_quat(0));
      transform_mb.setRotation(q_mb);
      tf::StampedTransform stamp_transform_mb(transform_mb, ros::Time::now(), "map_vis", "base_link_vis");

Pose3D a_bl;
Pose3D a_o;
Pose3D o_bl;


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
   */

  //Write out these data on socket



}
 
void Navigation::tf_broadcast_pose_arena(){

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(_world_pos(0),_world_pos(1),_world_pos(2)));
    tf::Quaternion q(_world_quat(1),_world_quat(2),_world_quat(3),_world_quat(0));
    transform.setRotation(q);
    tf::StampedTransform stamp_transform(transform, ros::Time::now(), "map_vis", "base_link_vis");
    _broadcaster.sendTransform(stamp_transform);

}
 
void Navigation::tf_broadcast_odom_arena(){

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(_H_odom_arena(0,3),_H_odom_arena(1,3),_H_odom_arena(2,3)));
    Eigen::Vector4d quat;
    quat = utilities::rot2quat( _H_odom_arena.block<3,3>(0,0) );
    tf::Quaternion q( quat(1),quat(2),quat(3),quat(0) );
    transform.setRotation(q);
    tf::StampedTransform stamp_transform(transform, ros::Time::now(), "map_vis", "odom_vis");
    _broadcaster.sendTransform(stamp_transform);

}

void Navigation::tf_broadcast_pose_odom(){

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(_world_pos_odom(0),_world_pos_odom(1),_world_pos_odom(2)));
    tf::Quaternion q(_world_quat_odom(1),_world_quat_odom(2),_world_quat_odom(3),_world_quat_odom(0));
    transform.setRotation(q);
    tf::StampedTransform stamp_transform(transform, ros::Time::now(), "odom_vis", "base_link_vis");
    _broadcaster.sendTransform(stamp_transform);

}



