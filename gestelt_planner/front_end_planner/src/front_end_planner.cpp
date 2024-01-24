#include <front_end_planner/front_end_planner.h>

void FrontEndPlanner::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{ 
  ROS_INFO("Initialized front end planner");
  double planner_freq;
  pnh.param("front_end/planner_frequency", planner_freq, -1.0);
  pnh.param("front_end/goal_tolerance", squared_goal_tol_, -1.0);

  int front_end_max_itr;
  pnh.param("front_end/max_iterations", front_end_max_itr, -1);
  
  SphericalSFC::SphericalSFCParams sfc_params; 
  pnh.param("sfc/max_iterations", sfc_params.max_itr, -1);
  pnh.param("sfc/max_sample_points", sfc_params.max_sample_points, -1);
  pnh.param("sfc/mult_stddev_x", sfc_params.mult_stddev_x, -1.0);
  pnh.param("sfc/W_intersect_vol", sfc_params.W_intersect_vol, -1.0);
  pnh.param("sfc/W_cand_vol", sfc_params.W_cand_vol, -1.0);
  pnh.param("sfc/W_cand_vol", sfc_params.W_cand_vol, -1.0);

  pnh.param("sfc/min_sphere_vol", sfc_params.min_sphere_vol, -1.0);
  pnh.param("sfc/max_sphere_vol", sfc_params.max_sphere_vol, -1.0);
  pnh.param("sfc/min_sphere_intersection_vol", sfc_params.min_sphere_intersection_vol, -1.0);

  pnh.param("sfc/avg_vel", sfc_params.avg_vel, 3.0);
  pnh.param("sfc/max_vel", sfc_params.max_vel, 1.5);

  squared_goal_tol_ *= squared_goal_tol_; 
  
  odom_sub_ = nh.subscribe("odom", 5, &FrontEndPlanner::odometryCB, this);
  goal_sub_ = nh.subscribe("planner/goals", 5, &FrontEndPlanner::goalsCB, this);

  debug_start_sub_ = nh.subscribe("debug/plan_start", 5, &FrontEndPlanner::debugStartCB, this);
  debug_goal_sub_ = nh.subscribe("debug/plan_goal", 5, &FrontEndPlanner::debugGoalCB, this);

  plan_on_demand_sub_ = nh.subscribe("plan_on_demand", 5, &FrontEndPlanner::planOnDemandCB, this);

  front_end_plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("plan_viz", 10);
  closed_list_viz_pub_ = nh.advertise<visualization_msgs::Marker>("closed_list_viz", 10);

  sfc_spherical_viz_pub_ = nh.advertise<visualization_msgs::Marker>("sfc_spherical", 10);

  sfc_p_cand_viz_pub_ = nh.advertise<visualization_msgs::Marker>("sfc_cand_points", 10);
  sfc_dist_viz_pub_ = nh.advertise<visualization_msgs::Marker>("sfc_dist", 10);
  sfc_waypoints_viz_pub_ = nh.advertise<visualization_msgs::Marker>("sfc_waypoints", 10);

  // plan_timer_ = nh.createTimer(ros::Duration(1/planner_freq), &FrontEndPlanner::planTimerCB, this);

  // Initialize map
  map_.reset(new GridMap);
  map_->initMap(nh, pnh);

  // Initialize front end planner 
  front_end_planner_ = std::make_unique<AStarPlanner>(map_);
  front_end_planner_->addVizPublishers(closed_list_viz_pub_);

  sfc_generation_ = std::make_unique<SphericalSFC>(map_, sfc_params);

  sfc_generation_->addVizPublishers(sfc_p_cand_viz_pub_, sfc_dist_viz_pub_, sfc_spherical_viz_pub_, sfc_waypoints_viz_pub_);
}

/**
 * Timer Callbacks
*/

void FrontEndPlanner::planTimerCB(const ros::TimerEvent &e)
{
  generatePlan();
}

bool FrontEndPlanner::generatePlan(){
  
  // Check if waypoint queue is empty
  if (waypoints_.empty()){
    return false;
  }

  if (isGoalReached(cur_pos_, waypoints_.nextWP())){
    waypoints_.popWP();
    return false;
  }

  // Generate a front-end path
  start_pos_ = cur_pos_;
  goal_pos_ = waypoints_.nextWP();

  ros::Time front_end_plan_start_time = ros::Time::now();

  if (!front_end_planner_->generatePlan(start_pos_, goal_pos_)){
    ROS_ERROR("[FrontEndPlanner] Path generation failed!");
    viz_helper::publishVizCubes(front_end_planner_->getClosedList(), "world", closed_list_viz_pub_);
    return false;
  }
  double front_end_plan_time_ms = (ros::Time::now() - front_end_plan_start_time).toSec() * 1000;

  std::vector<Eigen::Vector3d> front_end_path = front_end_planner_->getPathPos();
  std::vector<Eigen::Vector3d> closed_list = front_end_planner_->getClosedList();

  // Publish front end plan
  viz_helper::publishVizSpheres(front_end_path, "world", front_end_plan_viz_pub_) ;
  viz_helper::publishVizCubes(closed_list, "world", closed_list_viz_pub_);

  ros::Time sfc_plan_start_time = ros::Time::now();

  // Generate safe flight corridor from front end path
  if (!sfc_generation_->generateSFC(front_end_path)){
    return false;
  }
  double sfc_plan_time_ms = (ros::Time::now() - sfc_plan_start_time).toSec() * 1000;

  std::vector<SphericalSFC::Sphere> sfc_spheres = sfc_generation_->getSFCSpheres();

  logInfo(string_format("[FrontEndPlanner]: Front-end Planning Time: %f ms", front_end_plan_time_ms));
  logInfo(string_format("[FrontEndPlanner]: SFC Planning Time: %f ms", sfc_plan_time_ms));
  logInfo(string_format("[FrontEndPlanner]: Number of waypoints in front-end path: %ld", front_end_path.size()));
  logInfo(string_format("[FrontEndPlanner]: Size of closed list (expanded nodes): %ld", closed_list.size()));
  logInfo(string_format("[FrontEndPlanner]: Number of spheres in SFC Spherical corridor: %ld", sfc_spheres.size()));

  return true;
}

/**
 * Subscriber Callbacks
*/

/**
 * @brief Callback to generate plan on demand
 * 
 * @param msg 
 */
void FrontEndPlanner::planOnDemandCB(const std_msgs::EmptyConstPtr& msg){
  ROS_INFO("[FrontEndPlanner]: Planning on demand triggered! from (%f, %f, %f) to (%f, %f, %f)",
    cur_pos_(0), cur_pos_(1), cur_pos_(2),
    waypoints_.nextWP()(0), waypoints_.nextWP()(1), waypoints_.nextWP()(2)
  );
  generatePlan();
}

void FrontEndPlanner::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
  // TODO Add mutex 
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
  cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
}

void FrontEndPlanner::goalsCB(const gestelt_msgs::GoalsConstPtr &msg)
{
  if (msg->transforms.size() <= 0)
  {
    logError("[FrontEndPlanner]: Received empty waypoints");
    return;
  }
  if (msg->header.frame_id != "world" && msg->header.frame_id != "map" )
  {
    logError("[FrontEndPlanner]: Only waypoint goals in 'world' or 'map' frame are accepted, ignoring waypoints.");
    return;
  }

  std::vector<Eigen::Vector3d> wp_vec;

  for (auto& pos : msg->transforms) {
    // Transform received waypoints from world to UAV origin frame
    wp_vec.push_back(Eigen::Vector3d{pos.translation.x, pos.translation.y, pos.translation.z});
  }

  waypoints_.addMultipleWP(wp_vec);
}

void FrontEndPlanner::debugStartCB(const geometry_msgs::PoseConstPtr &msg)
{
  ROS_INFO("[FrontEndPlanner]: Received debug start (%f, %f, %f)", 
        msg->position.x,
        msg->position.y,
        msg->position.z);
  cur_pos_ = Eigen::Vector3d{
        msg->position.x,
        msg->position.y,
        msg->position.z};
}

void FrontEndPlanner::debugGoalCB(const geometry_msgs::PoseConstPtr &msg)
{
  ROS_INFO("[FrontEndPlanner]: Received debug goal (%f, %f, %f)", 
        msg->position.x,
        msg->position.y,
        msg->position.z);
  waypoints_.reset();
  waypoints_.addWP(Eigen::Vector3d{
        msg->position.x,
        msg->position.y,
        msg->position.z});
}

/* Planning helper methods */

/* Checking methods */

bool FrontEndPlanner::isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal){
  if ((pos - goal).squaredNorm() < squared_goal_tol_){
    return true;
  }

  return false;
}

bool FrontEndPlanner::isPlanFeasible(const Eigen::Vector3d& waypoints){
  // Check occupancy of every waypoint
  return true;
}