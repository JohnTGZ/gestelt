#ifndef _A_STAR_PLANNER_H_
#define _A_STAR_PLANNER_H_

#include <jps.hpp>
#include <ros/ros.h>

#include <chrono>

class JPS : public PlannerBase
{
public:

  struct JPSParams{
    int max_iterations; // Maximum iterations for Astar to run
    double tie_breaker;
    bool debug_viz; // Publish visualization messages for debugging 
    int cost_function_type; // Type of cost function to use
  }; // struct SphericalSFCParams

  JPS(std::shared_ptr<GridMap> map, const JPSParams& jps_params);

  /**
   * @brief Clear closed, open list and reset planning_successful flag for new plan generation
   * 
   */
  void reset();

  /**
   * @brief Generate a new plan. 
   * 
   * @param start_pos 
   * @param goal_pos 
   * @return true 
   * @return false 
   */
  bool generatePlan(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos);

  /**
   * @brief Get path with positions
   * 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> getPathPos();

private:
  std::vector<Eigen::Vector3d> path_pos_; // Path in terms of 3d position

  /* Params */
  // const double tie_breaker_ = 1.0 + 1.0 / 10000; 
  JPSParams jps_params_;

  std::shared_ptr<GridMap> map_;
};

#endif // _A_STAR_PLANNER_H_