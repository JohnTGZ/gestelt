#include <global_planner/jps.h>

JPS::JPS(std::shared_ptr<GridMap> map, const JPSParams& jps_params):
    jps_params_(jps_params)
{
    map_ = map;
}

void JPS::reset()
{
    path_pos_.clear();
}

bool JPS::generatePlan(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos)
{
    reset();

    // define start and goal
    Eigen::Vector3d start(start_arg[0], start_arg[1], start_arg[2]);
    Eigen::Vector3d goal(goal_arg[0], goal_arg[1], goal_arg[2]);

    // create global planner
    path_finding_util::GlobalPlanner global_planner;
    path_pos_ = global_planner.PlanJPS(start, goal, map_);

    // return
    return true;
}

/**
 * @brief Get successful plan in terms of path positions
 *
 * @return std::vector<Eigen::Vector3d>
 */
std::vector<Eigen::Vector3d> JPS::getPathPos()
{
    return path_pos_;
}
