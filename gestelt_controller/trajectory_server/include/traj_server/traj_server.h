#ifndef _TRAJECTORY_SERVER_H_
#define _TRAJECTORY_SERVER_H_

#include <numeric>
#include <deque>

#include <Eigen/Geometry> 

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <gestelt_msgs/Command.h>
#include <gestelt_msgs/CommanderState.h>
#include <gestelt_msgs/ExecTrajectory.h>

#include <visualization_msgs/Marker.h>

using namespace Eigen;

/* State machine  */
enum ServerState
{
  INIT,
  IDLE,
  TAKEOFF,
  LAND,
  HOVER,
  MISSION,
  E_STOP,
};

/* State machine events */
enum ServerEvent
{
  TAKEOFF_E,        // 0
  LAND_E,           // 1
  MISSION_E,        // 2
  HOVER_E,          // 3
  E_STOP_E,         // 4
  EMPTY_E,          // 5
};

enum TrajMode
{
  POLYTRAJ,                 // From egoswarm
  MULTIDOFJOINTTRAJECTORY,  // From mav_trajectory_generation
};

template<typename ... Args>
std::string str_fmt( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

struct SafetyLimits{
  double min_x{0}, max_x{0};
  double min_y{0}, max_y{0};
  double min_z{0}, max_z{0};
};

class TrajectoryServer{
public:
  /* Initialization methods */
  void init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private: // Class Methods

  /* ROS Callbacks */

  /**
   * @brief Callback for trajectory points from mav_trajectory_generation  
   */
  void execTrajCb(const gestelt_msgs::ExecTrajectory::ConstPtr &msg);

  /**
   * @brief Callback for Mavros state 
   */
  void UAVStateCb(const mavros_msgs::State::ConstPtr &msg);

  /**
   * @brief Callback for UAV Pose
   */
  void UAVPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /**
   * @brief Callback for UAV Odometry
   */
  void UAVOdomCB(const nav_msgs::Odometry::ConstPtr &msg);

  /**
   * @brief Callback for externally triggered server events
   */
  void serverCommandCb(const gestelt_msgs::Command::ConstPtr & msg);

  /**
   * @brief Callback for externally triggered server events
   */
  void swarmServerCommandCb(const std_msgs::Int8::ConstPtr & msg);

  /**
   * Timer callback to extract PVA commands from subscribed plan for executing trajectory.
   * It will determine what sort of trajectory to execute (takeoff, landing, hover, mission etc.)
   * based on the current state of the state machine.
  */
  void execTrajTimerCb(const ros::TimerEvent &e);

  /**
   * Timer callback to tick State Machine
   * This callback should only ever handle transitions between states,
   * It should not attempt to call any trajectory execution function, this 
   * should be left to the execTrajTimerCb callback.
  */
  void tickServerStateTimerCb(const ros::TimerEvent &e);

  /**
   * Timer callback to publish debug data such as UAV path, tracking error etc.
  */
  void debugTimerCb(const ros::TimerEvent &e);

  /* Trajectory execution methods */

  /**
   * Execute take off 
  */
  void execTakeOff();

  /**
   * Execute hover
  */
  void execHover();

  /**
   * Execute landing
  */
  void execLand();

  /**
   * Execute Mission Trajectory
  */
  void execMission();

  /* Conditional checking methods */

  /**
   * Return true if a mission is currently being executed
  */
  inline bool isExecutingMission()
  {
    // Returns true if time since last trajectory message is within timeout,
    // meaning that mission trajectory messages are still being received.
    // Else, return false
    return (ros::Time::now() - last_traj_msg_time_).toSec() < traj_msg_timeout_;
  }

  /**
   * Check if landing execution is complete
  */
  inline bool isLanded()
  {
    // Check that difference between desired landing height and current UAV position
    // is within tolerance 
    return abs(uav_pose_.pose.position.z - landed_height_) < take_off_landing_tol_;
  }

  /**
   * Check if take off execution is complete
  */
  inline bool isTakenOff()
  {
    return abs(uav_pose_.pose.position.z - takeoff_height_) < take_off_landing_tol_;
  }

  /* Publisher methods */

  /**
  * @brief Publish PVA (Position, Velocity, Acceleration) commands
  * 
  * @param p Position
  * @param v Velocity
  * @param a Acceleration
  * @param j Jerk
  * @param yaw Yaw
  * @param yaw_rate Yaw rate 
  * @param type_mask Type mask to mask the commands to ignore
  */
  void publishCmd(
    Vector3d p, Vector3d v, Vector3d a, 
    Vector3d j, double yaw, double yaw_rate, 
    uint16_t type_mask = 0);

  /* Helper methods */

  /**
   * Send request for PX4 to switch to offboard mode and arm
  */
  bool toggleOffboardMode(bool toggle);

  /**
   * Checks if UAV state is:
   * 1. In OFFBOARD mode 
   * 2. ARMED
  */
  bool isUAVReady() {
    return (uav_current_state_.mode == "OFFBOARD") 
      && uav_current_state_.armed;
  }

  /**
   * Checks if UAV state is:
   * 1. In "AUTO.LOITER" mode 
   * 2. DISARMED
  */
  bool isUAVIdle() {
    return (uav_current_state_.mode == "AUTO.LOITER") 
      && !uav_current_state_.armed;
  }

  /**
   * @brief Check if the given position is within the position safety limits
   * 
   * @param position_limits 
   * @param p 
   * @return true 
   * @return false 
   */
  bool checkPositionLimits(SafetyLimits position_limits, Vector3d p);

  /**
   * @brief Convert geometry_msgs/Vector3 to Eigen::Vector3d
   * 
   * @param geom_vect 
   * @param eigen_vect 
   */
  void geomMsgsVector3ToEigenVector3(const geometry_msgs::Vector3& geom_vect, Eigen::Vector3d& eigen_vect);

  /**
   * @brief Convert from quaternion to Euler angles (roll, pitch, yaw)
   * 
   * @param quat 
   * @return Eigen::Vector3d Euler angles (roll, pitch, yaw)
   */
  Eigen::Vector3d quaternionToRPY(const geometry_msgs::Quaternion& quat);

  /* FSM Methods */

  /** @brief StateToString interprets the input server state **/
  inline const std::string StateToString(ServerState state)
  {
      switch (state)
      {
          case ServerState::INIT:     return "INIT";
          case ServerState::IDLE:     return "IDLE";
          case ServerState::TAKEOFF:  return "TAKEOFF";
          case ServerState::LAND:     return "LAND";
          case ServerState::HOVER:    return "HOVER";
          case ServerState::MISSION:  return "MISSION";
          case ServerState::E_STOP:   return "E_STOP";
          default:                    return "[Unknown State]";
      }
  }

  /** @brief StateToString interprets the input server event **/
  inline const std::string EventToString(ServerEvent event)
  {
      switch (event)
      {
          case ServerEvent::TAKEOFF_E:  return "TAKEOFF";
          case ServerEvent::LAND_E:     return "LAND";
          case ServerEvent::MISSION_E:  return "MISSION";
          case ServerEvent::HOVER_E:    return "HOVER";
          case ServerEvent::EMPTY_E:    return "EMPTY";
          case ServerEvent::E_STOP_E:   return "E_STOP";
          default:                      return "[Unknown Event]";
      }
  }

  /* Send a server event to be processed by the state machine*/
  inline void setServerEvent(ServerEvent event)
  {
    // logInfo(str_fmt("Set server event: %s", EventToString(event).c_str()));
    server_event_ = event;
  }

  /* Retrieve a server event to be processed by the state machine,
  it will then set the event to be EMPTY, which prevents further processing*/
  ServerEvent getServerEvent()
  {
    // logInfo(str_fmt("Retrieved server event: %s", EventToString(server_event_).c_str()));
    ServerEvent event = server_event_;
    server_event_ = ServerEvent::EMPTY_E;  // Reset to empty

    return event;
  }

  /** Transition state machine to desired state.
   * This should ONLY be called within tickServerStateTimerCb.
   */
  void setServerState(ServerState des_state)
  {
    logInfo(str_fmt("Transitioning server state: %s -> %s", 
      StateToString(getServerState()).c_str(), StateToString(des_state).c_str()));

    server_state_ = des_state;
  }

  /** get current server state */
  ServerState getServerState()
  {
    return server_state_;
  }

private: // Member variables
  int drone_id_{0}; // ID of drone being commanded by trajectory server instance
  std::string origin_frame_; // frame that the drone originated from i.e. it's local pose is (0,0,0) w.r.t to this frame.

  /* Publisher  */
  ros::Publisher pos_cmd_raw_pub_; // Publisher of commands for PX4 
  ros::Publisher server_state_pub_; // Publisher of current uav and server state
  ros::Publisher vel_magnitude_pub_; // Publish velocity vector magnitude 
  
  /* Subscriber */
  ros::Subscriber exec_traj_sub_; // Subscriber for planner trajectory

  ros::Subscriber planner_hb_sub_; // Subscriber to planner heartbeat
  ros::Subscriber uav_state_sub_; // Subscriber to UAV State (MavROS)
  ros::Subscriber pose_sub_; // Subscriber to UAV State (MavROS)
  ros::Subscriber odom_sub_; // Subscriber to UAV State (MavROS)
  // TODO: make this a service server
  ros::Subscriber command_server_sub_; // Subscriber to trajectory server commands
  ros::Subscriber swarm_command_server_sub_; // Subscriber to swarm server commands

  /* Timer */
  ros::Timer exec_traj_timer_; // Timer to generate PVA commands for trajectory execution
  ros::Timer tick_state_timer_; // Timer to tick the state machine 
  ros::Timer debug_timer_; // Timer to publish debug data

  /** @brief Service clients **/
  ros::ServiceClient arming_client, set_mode_client; 

  /* Stored data*/
  ServerEvent server_event_{ServerEvent::EMPTY_E};
  ServerState server_state_{ServerState::INIT};
  mavros_msgs::State uav_current_state_;

  geometry_msgs::PoseStamped uav_pose_; // Current pose of UAV
  nav_msgs::Odometry uav_odom_; // Current odometry of UAV

  // Last received mission PVAJ (position, velocity, acceleration, Jerk)
  Eigen::Vector3d last_mission_pos_{0.0, 0.0, 0.0}, last_mission_vel_{0.0, 0.0, 0.0};
  Eigen::Vector3d last_mission_acc_{0.0, 0.0, 0.0}, last_mission_jerk_{0.0, 0.0, 0.0};
  // Last received mission yaw and yaw rate
  double last_mission_yaw_{0.0}, last_mission_yaw_dot_{0.0};

  /* Flags */ 
  ros::Time last_traj_msg_time_{0}; // Time of last trajectory message

  bool first_pose_{true};

  // Values set from mavros_msgs/PositionTarget message constants
  uint16_t IGNORE_POS; // Ignore position in typemask
  uint16_t IGNORE_VEL; // Ignore velocity in typemask
  uint16_t IGNORE_ACC; // Ignore acceleration in typemask
  uint16_t USE_FORCE; // Use force in typemask
  uint16_t IGNORE_YAW; // Ignore yaw in typemask
  uint16_t IGNORE_YAW_RATE; // Ignore yaw rate in typemask

  uint16_t mission_type_mask_{0}; // Current type mask

  std::mutex cmd_mutex_; // mutex for PVA Commands

  /* Params */ 
  std::string node_name_{"traj_server"};
  double pub_cmd_freq_; // Frequency to publish PVA commands
  double sm_tick_freq_; // Frequency of state machine ticks

  double takeoff_height_{0.0}; // Default height to take off to 
  double min_hover_height_{0.25};
  double landed_height_{0.1}; // We assume that the ground is even (z = 0)
  double take_off_landing_tol_{0.1}; // tolerance within desired take off or landing 

  double traj_msg_timeout_{0.2}; 

  int num_pose_msgs_{0};

  bool enable_safety_box_{true}; // Enables a safety bounding box and prevents quadrotor from exceeding this box. 
  SafetyLimits safety_box_;

private:
  void logInfo(const std::string& str){
    ROS_INFO("[%s] UAV_%i: %s", node_name_.c_str(), drone_id_, str.c_str());
  }

  void logWarn(const std::string& str){
    ROS_WARN("[%s] UAV_%i: %s", node_name_.c_str(), drone_id_, str.c_str());
  }

  void logError(const std::string& str){
    ROS_ERROR("[%s] UAV_%i: %s", node_name_.c_str(), drone_id_, str.c_str());
  }

  void logFatal(const std::string& str){
    ROS_FATAL("[%s] UAV_%i: %s", node_name_.c_str(), drone_id_, str.c_str());
  }

  void logInfoThrottled(const std::string& str, double period){
    ROS_INFO_THROTTLE(period, "[%s] UAV_%i: %s", node_name_.c_str(), drone_id_, str.c_str());
  }

  void logWarnThrottled(const std::string& str, double period){
    ROS_WARN_THROTTLE(period, "[%s] UAV_%i: %s", node_name_.c_str(), drone_id_, str.c_str());
  }

  void logErrorThrottled(const std::string& str, double period){
    ROS_ERROR_THROTTLE(period, "[%s] UAV_%i: %s", node_name_.c_str(), drone_id_, str.c_str());
  }

  void logFatalThrottled(const std::string& str, double period){
    ROS_FATAL_THROTTLE(period, "[%s] UAV_%i: %s", node_name_.c_str(), drone_id_, str.c_str());
  }

};

#endif //_TRAJECTORY_SERVER_H_