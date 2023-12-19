#include <traj_server/traj_server.h>

using namespace Eigen;

/* Initialization methods */

void TrajServer::init(ros::NodeHandle& nh)
{
  /////////////////
  /* ROS Params*/
  /////////////////
  nh.param("drone_id", drone_id_, 0);
  nh.param("origin_frame", origin_frame_, std::string("world"));

  // Operational params
  nh.param("traj_server/takeoff_height", takeoff_height_, 1.0);

  // Safety bounding box params
  nh.param("traj_server/enable_safety_box", enable_safety_box_, true);
  nh.param("traj_server/safety_box/max_x", safety_box_.max_x, -1.0);
  nh.param("traj_server/safety_box/min_x", safety_box_.min_x, -1.0);
  nh.param("traj_server/safety_box/max_y", safety_box_.max_y, -1.0);
  nh.param("traj_server/safety_box/min_y", safety_box_.min_y, -1.0);
  nh.param("traj_server/safety_box/max_z", safety_box_.max_z, -1.0);
  nh.param("traj_server/safety_box/min_z", safety_box_.min_z, -1.0);

  // Frequency params
  nh.param("traj_server/pub_cmd_freq", pub_cmd_freq_, 25.0); // frequency to publish commands
  double state_machine_tick_freq; // Frequency to tick the state machine transitions
  nh.param("traj_server/state_machine_tick_freq", state_machine_tick_freq, 50.0);
  double debug_freq; // Frequency to publish debug information
  nh.param("traj_server/debug_freq", debug_freq, 10.0);

  // Debug display params
  nh.param("traj_server/uav_pose_history_size", uav_pose_history_size_, 250);

  /////////////////
  /* Subscribers */
  /////////////////
  // Subscription to commands
  command_server_sub_ = nh.subscribe<gestelt_msgs::Command>("/traj_server/command", 10, &TrajServer::serverCommandCb, this);

  // Subscription to planner adaptor
  exec_traj_sub_ = nh.subscribe<gestelt_msgs::ExecTrajectory>("/planner_adaptor/exec_trajectory", 10, &TrajServer::execTrajCb, this);

  // Subscription to UAV (via MavROS)
  uav_state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &TrajServer::UAVStateCb, this);
  pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &TrajServer::UAVPoseCB, this);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &TrajServer::UAVOdomCB, this);

  /////////////////
  /* Publishers */
  /////////////////
  pos_cmd_raw_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 50);
  uav_path_pub_ = nh.advertise<nav_msgs::Path>("/uav_path_trajectory", 50);
  server_state_pub_ = nh.advertise<gestelt_msgs::CommanderState>("/traj_server/state", 50);

  /////////////////
  /* Service clients */
  /////////////////
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  /////////////////
  /* Timer callbacks */
  /////////////////
  exec_traj_timer_ = nh.createTimer(ros::Duration(1/pub_cmd_freq_), &TrajServer::execTrajTimerCb, this);
  tick_state_timer_ = nh.createTimer(ros::Duration(1/state_machine_tick_freq), &TrajServer::tickServerStateTimerCb, this);
  debug_timer_ = nh.createTimer(ros::Duration(1/debug_freq), &TrajServer::debugTimerCb, this);

  // Initialize ignore flags for mavros position target command
  IGNORE_POS = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ;
  IGNORE_VEL = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ;
  IGNORE_ACC = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ;
  USE_FORCE = mavros_msgs::PositionTarget::FORCE;
  IGNORE_YAW = mavros_msgs::PositionTarget::IGNORE_YAW;
  IGNORE_YAW_RATE = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

  logInfo("Initialized");
}

/* Subscriber Callbacks */

void TrajServer::execTrajCb(const gestelt_msgs::ExecTrajectory::ConstPtr &msg)
{
  if (getServerState() != ServerState::MISSION){ 
    logError("Executing Joint Trajectory while not in MISSION mode. Ignoring!");
    return;
  }

  last_traj_msg_time_ = ros::Time::now();

  std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);

  // std::string frame_id = msg->header.frame_id;
  
  mission_type_mask_ = msg->type_mask; 

  geomMsgsVector3ToEigenVector3(msg->transform.translation, last_mission_pos_);
  last_mission_yaw_ = quaternionToRPY(msg->transform.rotation)(2); // yaw

  geomMsgsVector3ToEigenVector3(msg->velocity.linear, last_mission_vel_);
  last_mission_yaw_dot_ = msg->velocity.angular.z; //yaw rate
  // ROS_INFO("received velocity: %f, %f, %f", last_mission_vel_(0), last_mission_vel_(1), last_mission_vel_(2));

  geomMsgsVector3ToEigenVector3(msg->acceleration.linear, last_mission_acc_);
  // ROS_INFO("received acceleration: %f, %f, %f", last_mission_acc_(0), last_mission_acc_(1), last_mission_acc_(2));

}

void TrajServer::UAVStateCb(const mavros_msgs::State::ConstPtr &msg)
{
  // logInfoThrottled(string_format("State: Mode[%s], Connected[%d], Armed[%d]", msg->mode.c_str(), msg->connected, msg->armed), 1.0);
  uav_current_state_ = *msg;
}

void TrajServer::UAVPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if (first_pose_){
    last_mission_pos_(0) = uav_pose_.pose.position.x;
    last_mission_pos_(1) = uav_pose_.pose.position.y;
    num_pose_msgs_++;
    if (num_pose_msgs_ > 100){
      first_pose_ = false;
      ROS_INFO("Taking off pose locked to (%f, %f)", last_mission_pos_(0), last_mission_pos_(1));
    }
  }

  uav_pose_ = *msg; 
  uav_poses_.push_back(uav_pose_);

  if (uav_poses_.size() > uint16_t(uav_pose_history_size_)) {
    uav_poses_.pop_front(); // Remove the oldest pose
  }

}

void TrajServer::UAVOdomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  uav_odom_ = *msg;
}

void TrajServer::serverCommandCb(const gestelt_msgs::Command::ConstPtr & msg)
{
  if (msg->command < 0 || msg->command > ServerEvent::EMPTY_E){
    logError("Invalid server command, ignoring...");
  }

  setServerEvent(ServerEvent(msg->command));
}

/* Timer Callbacks */

void TrajServer::execTrajTimerCb(const ros::TimerEvent &e)
{
  // has received vel value
  // ROS_INFO("execTrajTimerCb received velocity: %f, %f, %f", last_mission_vel_(0), last_mission_vel_(1), last_mission_vel_(2));

  switch (getServerState()){
    
    case ServerState::INIT:
      // Do nothing, drone is not initialized
      break;
    
    case ServerState::IDLE:
      // Do nothing, drone has not taken off
      break;
    
    case ServerState::TAKEOFF:
      execTakeOff();
      break;
    
    case ServerState::LAND:
      execLand();
      break;
    
    case ServerState::HOVER:
      execHover();
      break;
    
    case ServerState::MISSION:
      if (!isExecutingMission()){ // isExecutingMission() is true if last exec trajectory message did not exceed timeout
        logInfoThrottled("No Mission Received, waiting...", 2.0);
        execHover();
      }
      else {
        execMission();
      }
      break;

    case ServerState::E_STOP:
      // Do nothing, drone should stop all motors immediately
      execLand();
      break;
  }
}

void TrajServer::tickServerStateTimerCb(const ros::TimerEvent &e)
{
  // logInfoThrottled(string_format("Current Server State: [%s]", StateToString(getServerState()).c_str()), 1.0);

  switch (getServerState())
  {
    case ServerState::INIT:
      {
        // Wait for FCU Connection
        if (uav_current_state_.connected){
          logInfo("[INIT] Connected to flight stack!");
          setServerState(ServerState::IDLE);
        }
        else {
          logInfoThrottled("[INIT] Initializing Server, waiting for connection to FCU...", 2.0 );
        }

        break;
      }
    case ServerState::IDLE:
      // logInfoThrottled("[IDLE] Ready to take off", 5.0 );

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logInfo("[IDLE] UAV Attempting takeoff");
          setServerState(ServerState::TAKEOFF);
          break;
        case LAND_E:
          logWarn("[IDLE] IGNORED COMMAND. UAV has not taken off, unable to LAND");
          break;
        case MISSION_E:
          logWarn("[IDLE] IGNORED COMMAND. Please TAKEOFF first before setting MISSION mode");
          break;
        case HOVER_E:
          logWarn("[IDLE] IGNORED COMMAND. No mission to cancel");
          break;
        case E_STOP_E:
          logFatal("[IDLE] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }
      break;
    
    case ServerState::TAKEOFF:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          // logWarn("[TAKEOFF] IGNORED COMMAND. UAV already attempting taking off");
          break;
        case LAND_E:
          logInfo("[TAKEOFF] Attempting landing");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logWarn("[TAKEOFF] IGNORED COMMAND to MISSION. Wait until UAV needs to take off before accepting mission command");
          break;
        case HOVER_E:
          logWarn("[TAKEOFF] IGNORED COMMAND to HOVER. Currently TAKING OFF");
          break;
        case E_STOP_E:
          logFatal("[TAKEOFF] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (!isUAVReady()){
        logInfo("[TAKEOFF] Calling toggle offboard mode");
        toggleOffboardMode(true);
      }

      if (isTakenOff()){
        logInfo("[TAKEOFF] Take off complete");
        setServerState(ServerState::HOVER);
      }
      else {
        logInfoThrottled("[TAKEOFF] Taking off...", 1.0 );
      }

      break;
    
    case ServerState::LAND:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logInfo("[LAND] UAV taking off");
          setServerState(ServerState::TAKEOFF);
          break;
        case LAND_E:
          logWarn("[LAND] IGNORED COMMAND to LAND. UAV already attempting landing");
          break;
        case MISSION_E:
          logWarn("[LAND] IGNORED COMMAND to MISISON. UAV is landing, it needs to take off before accepting mission command");
          break;
        case HOVER_E:
          logWarn("[LAND] IGNORED COMMAND to HOVER. UAV needs to TAKE OFF before it can HOVER.");
          break;
        case E_STOP_E:
          logFatal("[LAND] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (isLanded()){
        logInfo("[LAND] Landing complete");
        setServerState(ServerState::IDLE);
      }
      else {
        logInfoThrottled("[LAND] landing...", 1.0);
      }

      setServerState(ServerState::IDLE);
      break;
    
    case ServerState::HOVER:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logWarn("[HOVER] IGNORED COMMAND to TAKE OFF. UAV already took off. Currently in [HOVER] mode");
          break;
        case LAND_E:
          logInfo("[HOVER] UAV is LANDING");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logInfo("[HOVER] UAV entering [MISSION] mode.");
          setServerState(ServerState::MISSION);
          break;
        case HOVER_E:
          logWarn("[HOVER] IGNORED COMMAND to HOVER. Already hovering...");
          break;
        case E_STOP_E:
          logFatal("[HOVER] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      break;
    
    case ServerState::MISSION:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          logWarn("[MISSION] IGNORED COMMAND. UAV already took off. Currently in [MISSION] mode");
          break;
        case LAND_E:
          logWarn("[MISSION] Mission cancelled! Landing...");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          logWarn("[MISSION] IGNORED COMMAND. UAV already in [MISSION] mode");
          break;
        case HOVER_E:
          logWarn("[MISSION] Mission cancelled! Hovering...");
          setServerState(ServerState::HOVER);
          break;
        case E_STOP_E:
          logFatal("[MISSION] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      break;

    case ServerState::E_STOP:
      logFatalThrottled("[E_STOP] Currently in E STOP State, please reset the vehicle and trajectory server!", 1.0);
      break;
  }
}

void TrajServer::debugTimerCb(const ros::TimerEvent &e){
  // Publish current Commander state
  gestelt_msgs::CommanderState state_msg;

  state_msg.drone_id = drone_id_;
  state_msg.traj_server_state = StateToString(getServerState());
  state_msg.planner_server_state = "UNIMPLEMENTED";
  state_msg.uav_state = uav_current_state_.mode;
  state_msg.armed = uav_current_state_.armed;

  server_state_pub_.publish(state_msg);

  // Publish UAV Pose history
  nav_msgs::Path uav_path;
  uav_path.header.stamp = ros::Time::now();
  uav_path.header.frame_id = origin_frame_; 
  uav_path.poses = std::vector<geometry_msgs::PoseStamped>(uav_poses_.begin(), uav_poses_.end());

  uav_path_pub_.publish(uav_path);
}

/* Trajectory execution methods */

void TrajServer::execLand()
{
  int type_mask = IGNORE_VEL | IGNORE_ACC | IGNORE_YAW_RATE ; // Ignore Velocity, Acceleration and yaw rate

  Eigen::Vector3d pos;
  pos << uav_pose_.pose.position.x, uav_pose_.pose.position.y, landed_height_;

  publishCmd( pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), 
              last_mission_yaw_, 0, 
              type_mask);
}

void TrajServer::execTakeOff()
{ 
  int type_mask = IGNORE_VEL | IGNORE_ACC | IGNORE_YAW_RATE ; // Ignore Velocity, Acceleration and yaw rate
  
  Eigen::Vector3d pos = last_mission_pos_;
  pos(2) = takeoff_height_;

  publishCmd( pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), 
              last_mission_yaw_, 0, 
              type_mask);
}

void TrajServer::execHover()
{
  int type_mask = IGNORE_VEL | IGNORE_ACC | IGNORE_YAW_RATE ; // Ignore Velocity, Acceleration and yaw rate
  Eigen::Vector3d pos = last_mission_pos_;

  // ensure that hover z position does not fall below 0.2m
  pos(2) = pos(2) < 0.2 ? 0.2 : pos(2);

  publishCmd( pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), 
              last_mission_yaw_, 0, 
              type_mask);
}

void TrajServer::execMission()
{
  std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
  // ROS_INFO("execMission() mission_vel: %f, %f, %f", last_mission_vel_(0), last_mission_vel_(1), last_mission_vel_(2));
  publishCmd( last_mission_pos_, last_mission_vel_, last_mission_acc_, last_mission_jerk_, 
              last_mission_yaw_, last_mission_yaw_dot_, 
              mission_type_mask_);
}

/* Publisher methods */

void TrajServer::publishCmd(
  Vector3d p, Vector3d v, Vector3d a, Vector3d j, double yaw, double yaw_rate, uint16_t type_mask)
{
  if (enable_safety_box_ && !checkPositionLimits(safety_box_, p)) {
    // If position safety limit check failed, switch to hovering mode
    setServerEvent(ServerEvent::HOVER_E);
  }

  mavros_msgs::PositionTarget pos_cmd;

  pos_cmd.header.stamp = ros::Time::now();
  pos_cmd.header.frame_id = origin_frame_;
  pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  pos_cmd.type_mask = type_mask;

  pos_cmd.position.x = p(0);
  pos_cmd.position.y = p(1);
  pos_cmd.position.z = p(2);
  pos_cmd.velocity.x = v(0);
  pos_cmd.velocity.y = v(1);
  pos_cmd.velocity.z = v(2);
  pos_cmd.acceleration_or_force.x = a(0);
  pos_cmd.acceleration_or_force.y = a(1);
  pos_cmd.acceleration_or_force.z = a(2);
  pos_cmd.yaw = yaw;
  pos_cmd.yaw_rate = yaw_rate;
  // ROS_INFO("Velocity for final command: %f, %f, %f", v(0), v(1), v(2));
  // ROS_INFO("Acceleration for final command: %f, %f, %f", a(0), a(1), a(2));
  pos_cmd_raw_pub_.publish(pos_cmd);
}

/* Helper methods */

bool TrajServer::toggleOffboardMode(bool toggle)
  {
    bool arm_val = false;
    std::string set_mode_val = "AUTO.LOITER"; 
    if (toggle){
      arm_val = true;
      set_mode_val = "OFFBOARD"; 
    }

    auto conditions_fulfilled = [&] () {
      return (toggle ? isUAVReady() : isUAVIdle());
    };

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm_val;

    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = set_mode_val;

    // Make sure takeoff is not immediately sent, 
    // this will help to stream the correct data to the program first.
    // Will give a 1sec buffer
    // ros::Duration(1.0).sleep();

    ros::Rate rate(pub_cmd_freq_);

    // send a few setpoints before starting
    for (int i = 0; ros::ok() && i < 10; i++)
    {
      execTakeOff();
      ros::spinOnce();
      rate.sleep();
    }
    ros::Time last_request_t = ros::Time::now();

    while (!conditions_fulfilled()){

      bool request_timeout = ((ros::Time::now() - last_request_t) > ros::Duration(2.0));

      if (uav_current_state_.mode != set_mode_val && request_timeout)
      {
        if (set_mode_client.call(set_mode_srv))
        {
          if (set_mode_srv.response.mode_sent){
            logInfo(string_format("Setting %s mode successful", set_mode_val.c_str()));
          }
          else {
            logInfo(string_format("Setting %s mode failed", set_mode_val.c_str()));
          }
        }
        else {
          logInfo("Service call to PX4 set_mode_client failed");
        }

        last_request_t = ros::Time::now();
      }
      else if (uav_current_state_.armed != arm_val && request_timeout) 
      {
        if (arming_client.call(arm_cmd)){
          if (arm_cmd.response.success){
            logInfo(string_format("Setting arm to %d successful", arm_val));
          }
          else {
            logInfo(string_format("Setting arm to %d failed", arm_val));
          }
        }
        else {
          logInfo("Service call to PX4 arming_client failed");
        }

        last_request_t = ros::Time::now();
      }
      ros::spinOnce();
      rate.sleep();        
    }

    return true;
  }

bool TrajServer::checkPositionLimits(SafetyLimits position_limits, Vector3d p){

  if (p(0) < position_limits.min_x || p(0) > position_limits.max_x){
    logError(string_format("Commanded x position (%f) exceeded x limits (%f-%f)", 
      p(0), position_limits.min_x, position_limits.max_x));

    return false;
  }
  else if (p(1) < position_limits.min_y || p(1) > position_limits.max_y) {

    logError(string_format("Commanded y position (%f) exceeded y limits (%f-%f)", 
      p(1), position_limits.min_y, position_limits.max_y));

    return false;
  }
  else if (p(2) < position_limits.min_z || p(2) > position_limits.max_z) {

    logError(string_format("Commanded z position (%f) exceeded z limits (%f-%f)", 
      p(2), position_limits.min_z, position_limits.max_z));

    return false;
  }

  return true;
}

void TrajServer::geomMsgsVector3ToEigenVector3(const geometry_msgs::Vector3& geom_vect, Eigen::Vector3d& eigen_vect){
  eigen_vect(0) = geom_vect.x;
  eigen_vect(1) = geom_vect.y;
  eigen_vect(2) = geom_vect.z;
}

Eigen::Vector3d TrajServer::quaternionToRPY(const geometry_msgs::Quaternion& quat){
  // Quaternionf q << quat.x, quat.y, quat.z, quat.w;
  Quaterniond q(quat.w, quat.x, quat.y, quat.z);

  Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2); // In roll, pitch and yaw

  return euler;
}

