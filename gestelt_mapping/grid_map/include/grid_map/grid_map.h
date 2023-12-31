#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
// #include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include <tf2/transform_datatypes.h>

// #include <swarm_benchmark/timebenchmark.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

struct MappingParameters
{
  /* map properties */
  Eigen::Vector3d map_origin_; // Origin of map
  Eigen::Vector3d global_map_size_; //  Size of global occupancy grid 
  Eigen::Vector3d local_map_size_; //  Size of local occupancy grid 

  double resolution_; // Voxel size for occupancy grid without inflation                  
  double inflation_; // Voxel size for occupancy grid with inflation
  int pose_type_; // Type of pose input (pose or odom)
  int sensor_type_; // Type of sensor (cloud or depth image)

  double pose_timeout_; // Timeout for pose update before emergency stop is activated

  /* camera parameters */
  double cx_, cy_, fx_, fy_, fx_inv_, fy_inv_; // Intrinsic camera parameters
  double k_depth_scaling_factor_; // Scaling factor for depth value of depth image
  double max_range;

  /* Cloud downsampler parameters */
  bool downsample_cloud_; // True if downsampling cloud before input to octree occupancy
  int depth_stride_; // Number of depth pixels to skip
  double voxel_size_; // Size of voxel for voxel grid filter

  /* visualization and computation time display */
  double ground_height_; // Lowest possible height (z-axis)

  std::string cam_frame_;
  std::string global_frame_; // frame id of global reference 
  std::string uav_origin_frame_; // frame id of UAV origin

  bool keep_global_map_{false}; // If true, octomap will not discard any nodes outside of the local map bounds. We will map the entire area but at the cost of additional memory.

};

// intermediate mapping data for fusion

struct MappingData
{
  // camera position in uav origin frame 
  Eigen::Vector3d cam_pos_{0.0, 0.0, 0.0};

  // Rotation matrix of camera to UAV origin frame
  Eigen::Matrix3d cam_to_origin_r_m_;

  // Transformation matrix of camera to body frame
  Eigen::Matrix4d cam2body_;
  // Transformation matrix of body to UAV origin frame
  Eigen::Matrix4d body2origin_;
  // Transformation matrix of camera to UAV origin frame
  Eigen::Matrix4d cam2origin_;

  Eigen::Vector3d local_map_min_; // minimum 3d bound of local map in (x,y,z)
  Eigen::Vector3d local_map_max_; // maximum 3d bound of local map in (x,y,z)

  // depth image data
  // cv::Mat depth_image_;

  // True if pose has been received
  bool has_pose_{false};

  // TODO: Use this to flag timeout
  // True if depth and odom has timed out
  bool flag_sensor_timeout_{false};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{

// Custom type definition for message filters
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
    SyncPolicyImageOdom;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
    SyncPolicyImagePose;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
    SyncPolicyCloudOdom;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped>
    SyncPolicyCloudPose;

typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

public:
  typedef std::shared_ptr<GridMap> Ptr;

  enum PoseType
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    TF = 3,
    INVALID_IDX = -10000
  };

  enum SensorType
  {
    SENSOR_CLOUD = 1,
    SENSOR_DEPTH = 2,
  };

  /* Initialization methods */

  GridMap() {}
  ~GridMap() {}

  // Reset map data
  void reset();

  // Initialize the GridMap class and it's callbacks
  void initMap(ros::NodeHandle &nh);

  // Get time benchmark shared pointer
  // void initTimeBenchmark(std::shared_ptr<TimeBenchmark> time_benchmark);
  
  /* Gridmap access methods */

  // True if given GLOBAL position is within the GLOBAL map boundaries, else False
  bool isInGlobalMap(const Eigen::Vector3d &pos);
  // True if given GLOBAL position is within the LOCAL map boundaries, else False
  bool isInLocalMap(const Eigen::Vector3d &pos);

  // Get occupancy value of given position in Occupancy grid
  int getOccupancy(const Eigen::Vector3d &pos);
  // Get occupancy value of given position in inflated Occupancy grid
  int getInflateOccupancy(const Eigen::Vector3d &pos);

  /* Gridmap conversion methods */

  // Create inflation sphere for inflating the octomap
  void setInflation(const double& inflation_radius, const double& map_res);

  // Get camera-to-global frame transformation
  void getCamToGlobalPose(const geometry_msgs::Pose &pose);
  
  // Take in point cloud as octree map. Transformation from camera-to-global frame is 
  // done here
  void cloudToCloudMap(const sensor_msgs::PointCloud2 &msg);

  // Take in depth image as octree map.  Transformation from camera-to-global frame is 
  // done here
  void depthToCloudMap(const sensor_msgs::ImageConstPtr &msg);

  // Convert PCL Point cloud to Octomap
  void pclToOctomapPC(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud , octomap::Pointcloud& octomap_cloud);
  
  // Retrieve occupied cells in Octree as a PCL Point cloud
  void octreeToPclPC(std::shared_ptr<octomap::OcTree> tree, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);

  // Update the local map
  void updateLocalMap();

  /** Helper methods */
  
  // Checks if camera pose is valid
  bool isPoseValid();

  // Get occupancy grid resolution
  double getResolution() { return mp_.resolution_; }

  // Get odometry depth timeout
  bool getPoseDepthTimeout() { return md_.flag_sensor_timeout_; }

  /** Publisher methods */
  void publishMap();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;
  std::string node_name_;

  /**
   * Subscriber Callbacks
  */

  // Subscriber callback to camera info 
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  // Subscriber callback to depth image and base_link odom
  void depthOdomCB( const sensor_msgs::ImageConstPtr &msg_img, 
                    const nav_msgs::OdometryConstPtr &msg_odom);

  // Subscriber callback to depth image and base_link pose
  void depthPoseCB(const sensor_msgs::ImageConstPtr &msg_img,
                    const geometry_msgs::PoseStampedConstPtr &msg_pose);

  // Subscriber callback to point cloud and odom
  void cloudOdomCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc, 
                    const nav_msgs::OdometryConstPtr &msg_odom);

  // Subscriber callback to point cloud and pose
  void cloudPoseCB(const sensor_msgs::PointCloud2ConstPtr &msg_pc,
                    const geometry_msgs::PoseStampedConstPtr &msg_pose);

  // Subscriber callback to point cloud and TF
  void cloudTFCB(const sensor_msgs::PointCloud2ConstPtr &msg_pc);

  /**
   * Timer Callbacks
  */

  /**
   * @brief This timer publishes a visualization of the occupancy grid
  */
  void visTimerCB(const ros::TimerEvent & /*event*/);


private: 
  ros::NodeHandle node_;

  /* ROS Publishers, subscribers and Timers */

  // Message filters for point cloud/depth camera and pose/odom
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;
  SynchronizerCloudPose sync_cloud_pose_;
  SynchronizerCloudOdom sync_cloud_odom_;
  
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

  // Message filters for point cloud and tf
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> tf_cloud_filter_;

  ros::Subscriber camera_info_sub_;
  ros::Subscriber cloud_only_sub_;

  ros::Publisher occ_map_pub_;
  ros::Timer vis_timer_; // Timer for visualization

  // TF transformation 
  tf2_ros::Buffer tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  /* Benchmarking */
  // std::shared_ptr<TimeBenchmark> time_benchmark_;

  /* Data structures for point clouds */
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_origin_;  // Point cloud local map in UAV origin frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_origin_;  // Point cloud global map in UAV Origin frame

  std::shared_ptr<octomap::OcTree> octree_; // Octree data structure

  pcl::VoxelGrid<pcl::PointXYZ> vox_grid_filter_; // Voxel filter
};


#endif //_GRID_MAP_H
