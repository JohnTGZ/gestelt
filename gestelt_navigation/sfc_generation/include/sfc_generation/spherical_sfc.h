#ifndef _SPHERICAL_SFC_H_
#define _SPHERICAL_SFC_H_

#include <sfc_generation/sfc_base.h>
#include "halton_enum.h"    // For deterministic sampling
#include "halton_sampler.h" // For deterministic sampling

#include "nanoflann.hpp" // For nearest neighbors queries
#include "KDTreeVectorOfVectorsAdaptor.h" // For nearest neighbors queries

#include <random>
#include <chrono>
#include <queue>
#include <chrono>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class SphericalSFC : public SFCBase
{
public: // Public structs

  struct Sphere {
    double radius{-1.0};
    double radius_sqr{-1.0};
    Eigen::Vector3d center{0.0, 0.0, 0.0};
    double score{-1.0};
    double volume{-1.0};

    Sphere(){}

    Sphere(const double& x, const double& y, const double& z, const double& radius):
      center(Eigen::Vector3d{x,y,z}), radius(radius), radius_sqr(radius*radius), volume(1.333 * M_PI * radius * radius * radius)
    {}

    Sphere(const Eigen::Vector3d& center, const double& radius):
      center(center), radius(radius), radius_sqr(radius*radius), volume(1.333 * M_PI * radius * radius * radius)
    {}

    // Copy constructor from shared_ptr
    Sphere(const std::shared_ptr<Sphere>& sphere): 
      radius(sphere->radius), 
      radius_sqr(sphere->radius_sqr), 
      center(sphere->center),
      score(sphere->score),
      volume(sphere->volume)
    {}

    /**
     * @brief Returns true if point is contained inside sphere (including at it's
     * boundary). Else return false
     * 
     * @param pt point to check
     * @return true 
     * @return false 
     */
    bool contains(const Eigen::Vector3d& pt) const
    {
      return (this->center - pt).squaredNorm() <= this->radius_sqr;
    }

    double getDiameter() const
    {
      return this->radius * 2;
    }

    double getVolume() const
    {
      return this->volume;
    }

    void setRadius(const double& radius)
    {
      this->radius = radius;
      this->radius_sqr = radius * radius;
      this->volume = 1.333 * M_PI * radius * radius * radius;
    }

    // Comparison operator between pointers
    struct CompareScorePtr
    {
      bool operator()(const std::shared_ptr<Sphere>& l_sphere, const std::shared_ptr<Sphere>& r_sphere)
      {
        return l_sphere->score < r_sphere->score;
      }
    };

  }; // struct Sphere

  struct SamplerNormal {
    std::default_random_engine gen; 

    std::normal_distribution<double> x_dis;
    std::normal_distribution<double> y_dis;
    std::normal_distribution<double> z_dis;

    SamplerNormal(){}

    SamplerNormal(const Eigen::Vector3d& mean, const Eigen::Vector3d& stddev)
    {
      x_dis = std::normal_distribution<double>(mean(0), stddev(0));
      y_dis = std::normal_distribution<double>(mean(1), stddev(1));
      z_dis = std::normal_distribution<double>(mean(2), stddev(2));
    }

    void setParams(const Eigen::Vector3d& mean, const Eigen::Vector3d& stddev, uint64_t seed = 0)
    {
      x_dis = std::normal_distribution<double>(mean(0), stddev(0));
      y_dis = std::normal_distribution<double>(mean(1), stddev(1));
      z_dis = std::normal_distribution<double>(mean(2), stddev(2));

      gen.seed(seed);
    }

    std::vector<Eigen::Vector3d> sample(const unsigned& num_samples) {
      std::vector<Eigen::Vector3d> samp_pts;
      for (unsigned i = 0; i < num_samples; ++i) // Iterate over samples in the pixel.
      {
        samp_pts.push_back(Eigen::Vector3d{x_dis(gen), y_dis(gen), z_dis(gen)});
      }

      return samp_pts;
    }

  }; // struct Sampler

  struct SamplerHalton {
    Halton_sampler halton_sampler_;
    Eigen::Vector3d mean_;
    Eigen::Vector3d stddev_;

    SamplerHalton()
    {
      halton_sampler_.init_faure();
    }

    void setParams(const Eigen::Vector3d& mean, const Eigen::Vector3d& stddev, uint64_t seed = 0)
    {
      mean_ = mean;
      stddev_ = stddev;
    }

    std::vector<Eigen::Vector3d> sample(const unsigned& num_samples) {
      std::vector<Eigen::Vector3d> samp_pts;

      for (unsigned i = 0; i < num_samples; ++i) // Iterate over samples in the pixel.
      {
        // Draw three components.
        auto samp_pt = Eigen::Vector3d{ 
            halton_sampler_.sample(0, i), 
            halton_sampler_.sample(1, i), 
            halton_sampler_.sample(2, i) };

        // Points are sampled from a value of 0.0 to 1.0, so we must center them at 0.5.
        samp_pt = samp_pt.array() - Eigen::Vector3d{0.5, 0.5, 0.5}.array();

        // Rescale the 3 components
        samp_pt = samp_pt.array() * stddev_.array();

        // Translate by the mean
        samp_pt = samp_pt + mean_;

        samp_pts.push_back(samp_pt);
      }

      return samp_pts;
    }

  }; // struct Sampler

  struct SphericalSFCParams{
    /* SFC Generation */
    int max_itr; // Corresponds to maximum number of spheres
    bool debug_viz; // If true, publish visualization for debugging

    /* Sampling */
    int max_sample_points; // Maximum allowed sampling points
    double mult_stddev_x; // Multiplier for x standard deviation in sampling 
    double mult_stddev_y; // Multiplier for x standard deviation in sampling 
    double mult_stddev_z; // Multiplier for x standard deviation in sampling 

    /* Scoring metric*/
    double W_cand_vol;    // Weight of candidate volume
    double W_intersect_vol; // Weight of intersection of volumes
    double W_progress; // Weight for progress along guide path

    double min_sphere_vol; // Minimum volume of sphere
    double max_sphere_vol; // Maximum volume of spheres

    // /* Time allocation */
    double max_vel; // Maximum velocity
    double max_acc; // Maximum acceleration

    double spherical_buffer; // Reduce sampled spherical corridors by this amount as a buffer

  }; // struct SphericalSFCParams

  struct SFCTrajectory{ // SFCTrajectory contains the spheres, trajectory waypoints and time allocation
    SFCTrajectory(){} // Default constructor

    /**
     * @brief Clear all data structures
     * 
     */
    void clear(){
      spheres.clear();
      segs_t_dur.clear();
      waypoints.clear();
    }

    Eigen::Vector3d const getStartPos() {
      if (waypoints.size() < 2){
        throw std::runtime_error("SFCTrajectory does not contain at least 2 waypoints");
      }
      return waypoints[0];
    }

    Eigen::Vector3d const getGoalPos() {
      if (waypoints.size() < 2){
        throw std::runtime_error("SFCTrajectory does not contain at least 2 waypoints");
      }
      return waypoints.back();
    }

    Eigen::VectorXd getSegmentTimeDurations() {
      Eigen::VectorXd segs_t_dur(segs_t_dur.size());

      for (size_t i = 0; i < segs_t_dur.size(); i++){
        segs_t_dur(i) = segs_t_dur[i];
      }

      return segs_t_dur;
    }

    std::vector<double> getSpheresRadii() {
      std::vector<double> spheres_radii;

      for (auto sphere : spheres){
        spheres_radii.push_back(sphere.radius);
      }

      return spheres_radii;
    }

    std::vector<Eigen::Vector3d> getSpheresCenter() {
      std::vector<Eigen::Vector3d> spheres_center;

      for (auto sphere : spheres){
        spheres_center.push_back(
          Eigen::Vector3d{sphere.center.x, sphere.center.y, sphere.center.z});
      }

      return spheres_center;
    }

    Eigen::MatrixXd getInnerWaypoints() {
      Eigen::MatrixXd inner_wps(3, waypoints.size()); // matrix of inner waypoints

      for (size_t i = 1; i < waypoints.size()-1; i++){
        inner_wps.col(i) = waypoints[i];
      }

      return inner_wps;
    }

    int getNumSegments() const{
      return spheres.size();
    }

    std::vector<SphericalSFC::Sphere> spheres;  // Vector of Spheres 
    std::vector<Eigen::Vector3d> waypoints;     // Vector of 3d waypoint positions {p1, p2, ... p_M+1}
    std::vector<double> segs_t_dur;          // Vector of time durations of each segment {t_1, t_2, ..., t_M}

  }; // struct SFCTrajectory

public:
  SphericalSFC(std::shared_ptr<GridMap> grid_map, const SphericalSFCParams& sfc_params);

  /**
   * @brief Reset existing data structures used during planning
   * 
   */
  void reset();

  /**
   * @brief Clear published visualizations
   * 
   */
  void clearVisualizations();
  
  /**
   * @brief Add publishers for visualization
   * 
   * @param p_cand_viz_pub 
   * @param dist_viz_pub 
   * @param samp_dir_vec_pub 
   * @param sfc_spherical_viz_pub 
   * @param sfc_waypoints_viz_pub 
   */
  void addVizPublishers(
    ros::Publisher& p_cand_viz_pub, 
    ros::Publisher& dist_viz_pub, ros::Publisher& samp_dir_vec_pub,
    ros::Publisher& sfc_spherical_viz_pub, ros::Publisher&  sfc_waypoints_viz_pub
  );

  /**
   * @brief Generate a spherical safe flight corridor given a path
   * 
   * @param path 
   * @return true 
   * @return false 
   */
  bool generateSFC(const std::vector<Eigen::Vector3d> &path);

  /* Getter methods */

  SphericalSFC::SFCTrajectory const getSFCTrajectory(){
    return sfc_traj_;
  }

  void getSFCTrajectoryDebug(
    std::vector<std::vector<SphericalSFC::Sphere>>& sfc_sampled_spheres,
    std::vector<Eigen::Vector3d>& samp_dir_vec,
    std::vector<Eigen::Vector3d>& guide_points_vec)
  {
    sfc_sampled_spheres = sfc_sampled_spheres_;
    samp_dir_vec = samp_dir_vec_;
    guide_points_vec = guide_points_vec_;
  }


private: // Private methods

  /**
   * @brief Initialize sampler 
   * 
   */
  void initSampler();

  /**
   * @brief Generate a free space sphere the given point. It's radius is determined by
   * the distance to the nearest obstacle
   * 
   * @param point 
   * @param sphere 
   * @return true 
   * @return false 
   */
  bool generateFreeSphere(const Eigen::Vector3d& point, Sphere& B);

  /**
   * @brief Get nearest point on guide path outside the given sphere
   * 
   * @param path 
   * @param start_idx 
   * @param B_prev Previous sphere 
   * @return true 
   * @return false 
   */
  bool getForwardPointOnPath(
    const std::vector<Eigen::Vector3d> &path, size_t& start_idx, const Sphere& B_prev);

  /**
   * @brief Sample a batch of spheres
   * 
   * @param pt_guide 
   * @param B_cur 
   * @return true 
   * @return false 
   */
  bool BatchSample(const Eigen::Vector3d& pt_guide, Sphere& B_cur);

  /**
   * @brief Get the center of the curve of intersection made between spheres B_a and B_b
   * 
   * @param B_a 
   * @param B_b 
   * @return Eigen::Vector3d 
   */
  Eigen::Vector3d getIntersectionCenter(const Sphere& B_a, const Sphere& B_b);

  /**
   * @brief Compute the score of the candidate sphere, given the previous sphere in the SFC
   * 
   * @param B_cand 
   * @param B_prev 
   * @return double 
   */
  double computeCandSphereScore(Sphere& B_cand, Sphere& B_prev);

  /**
   * @brief Transform points about the origin
   * 
   * @param pts 
   * @param origin 
   * @param rot_mat 
   */
  void transformPoints(std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d origin, const Eigen::Matrix<double, 3, 3>& rot_mat);

  /**
   * @brief Get the volume of intersection between 2 spheres
   * 
   * @param B_a 
   * @param B_b 
   * @return double Returns -1 if 2 spheres do not intersect, else return volume of intersection 
   */
  double getIntersectingVolume(Sphere& B_a, Sphere& B_b);

  /**
   * @brief Get rotation matrix that aligns vector z to vector d
   * 
   * @param z 
   * @param d 
   * @return Eigen::Matrix<double, 3, 3> 
   */
  Eigen::Matrix<double, 3, 3> rotationAlign(const Eigen::Vector3d & z, const Eigen::Vector3d & d);

  /**
   * @brief Compute a SFC Trajectory given a sequence of intersecting spheres
   * 
   * @param sfc_spheres 
   * @param goal_pos 
   * @param sfc_traj 
   */
  void computeSFCTrajectory(
    const std::vector<SphericalSFC::Sphere>& sfc_spheres, 
    const Eigen::Vector3d& start_pos, 
    const Eigen::Vector3d& goal_pos, 
    SphericalSFC::SFCTrajectory& sfc_traj);

  /* Visualization methods */

  void publishVizPoints(const std::vector<Eigen::Vector3d>& pts, 
                        ros::Publisher& publisher, Eigen::Vector3d color = Eigen::Vector3d{0.0, 0.0, 0.0}, 
                        double radius = 0.025, const std::string& frame_id = "world");

  void publishVizPiecewiseTrajectory( const std::vector<Eigen::Vector3d>& pts, 
                                      ros::Publisher& publisher, const std::string& frame_id = "world");

  void publishVizSphericalSFC(const std::vector<SphericalSFC::Sphere>& sfc_spheres, 
                              ros::Publisher& publisher, const std::string& frame_id = "world");

  visualization_msgs::Marker createArrow(
    const Eigen::Vector3d& start_pt, const Eigen::Vector3d& dir_vec, 
    const std::string& frame_id, const int& id);

  visualization_msgs::Marker createVizSphere( const Eigen::Vector3d& center, const double& diameter, 
                                              const std::string& frame_id, const int& id);

  visualization_msgs::Marker createVizEllipsoid(const Eigen::Vector3d& center, const Eigen::Vector3d& stddev, const Eigen::Quaterniond& orientation, const std::string& frame_id, const int& id);

private: // Private members
  SamplerHalton sampler_; // Sampler for sampling SFC waypoints
  // SamplerNormal sampler_; // Sampler for sampling SFC waypoints

  ros::Publisher p_cand_viz_pub_; // (visualization_msgs::Marker) Visualization of sampling points
  ros::Publisher dist_viz_pub_; // (visualization_msgs::MarkerArray) Visualization of sampling distribution
  ros::Publisher sfc_spherical_viz_pub_; // (visualization_msgs::MarkerArray) Visualization of spherical SFC
  ros::Publisher sfc_waypoints_viz_pub_; // (visualization_msgs::Marker) Visualization of trajectory waypoints
  ros::Publisher samp_dir_vec_pub_; // (visualization_msgs::MarkerArray) Visualization of direction vectors used for sampling

  /* Params */
  int itr_; // Iteration number
  SphericalSFCParams sfc_params_; // SFC parameters
  
  /* Data structs */
  std::shared_ptr<GridMap> grid_map_; 
  std::vector<SphericalSFC::Sphere> sfc_spheres_; // Waypoints of the spherical flight corridor
  SphericalSFC::SFCTrajectory sfc_traj_;          // SFC Trajectory 

  std::unique_ptr<KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3d>, double>>   
    guide_path_kdtree_; // KD Tree for guide path

  /* Data for Visualization */
  std::vector<Eigen::Vector3d> p_cand_vec_hist_; // history of candidate points
  visualization_msgs::MarkerArray sampling_dist_hist_; // history of sampling distributions (1 s.d.)
  visualization_msgs::MarkerArray samp_dir_vec_hist_; // history of sampling distributions (1 s.d.)

  std::vector<std::vector<SphericalSFC::Sphere>> sfc_sampled_spheres_ ; // Outer index is segment, each segment contains a number of sampled spheres
  std::vector<Eigen::Vector3d> samp_dir_vec_; // vector of sampling vectors
  std::vector<Eigen::Vector3d> guide_points_vec_; // vector of sampling guide points

  std::vector<Eigen::Vector3d> front_end_path_; // front_end path

}; // class SphericalSFC

#endif // _SPHERICAL_SFC_H_