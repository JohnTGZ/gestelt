#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <grid_map/grid_map.h>
#include <path_searching/dyn_a_star.h>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/planning_visualization.h>
#include "optimizer/lbfgs.hpp"
#include "poly_traj_utils.hpp"

namespace ego_planner
{

  class ConstraintPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // p of the {p,v} pair. The point at the start of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // v of the {p,v} pair. Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.

    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();

      points.resize(3, size_set);
      base_point.resize(cp_size);
      direction.resize(cp_size);
      flag_temp.resize(cp_size);
    }

    void segment(ConstraintPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= cp_size || points.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize_cp(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.cp_size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];
      }
    }

    /**
     * @brief Return idx of the 2/3 waypoint
     * 
     * @param points 
     * @param touch_goal 
     * @return int 
     */
    static int two_thirds_id(Eigen::MatrixXd &points, const bool touch_goal)
    {
      return touch_goal ? points.cols() - 1 : points.cols() - 1 - (points.cols() - 2) / 3;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {

  private:
    std::shared_ptr<GridMap> grid_map_;
    AStar::Ptr a_star_;
    PlanningVisualization::Ptr visualization_;

    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    ConstraintPoints cps_;
    // PtsChk_t pts_check_;

    int drone_id_;
    int cps_num_perPiece_;   // number of distinctive constraint points per piece
    int variable_num_;       // optimization variables
    int piece_num_;          // poly traj piece numbers
    int iter_num_;           // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in swarm
    bool touch_goal_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_, wei_obs_soft_;                               // obstacle weight
    double wei_swarm_;                                            // swarm weight
    double wei_feas_;                                             // feasibility weight
    double wei_sqrvar_;                                           // squared variance weight
    double wei_time_;                                             // time weight
    double wei_formation_;                                            // formation weight
    double obs_clearance_, obs_clearance_soft_, swarm_clearance_; // safe distance
    double max_vel_, max_acc_;                                    // dynamic limits

    int formation_num_;
    Eigen::MatrixXd formation_;
    Eigen::Vector3d FStart_, FEnd_;                                 //first start pt, final end pt

    double t_now_;

  public:
    PolyTrajOptimizer(){}
    ~PolyTrajOptimizer() {}

    enum CHK_RET
    {
      OBS_FREE,
      ERR,
      FINISH
    };

    /* set variables */
    void setParam(ros::NodeHandle &pnh);
    void setEnvironment(const std::shared_ptr<GridMap> &map);
    void setVisualizer(PlanningVisualization::Ptr vis);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);
    void setIfTouchGoal(const bool touch_goal);
    void setFStartFEnd(const Eigen::Vector3d &formation_start_pt, const Eigen::Vector3d &formation_end_pt);
    void setConstraintPoints(ConstraintPoints cps);

    /* helper functions */
    const ConstraintPoints &getControlPoints(void) { return cps_; }

    /**
     * Returns the minimum jerk optimizer object
    */
    const poly_traj::MinJerkOpt &getMinJerkOpt(void) { return jerkOpt_; }

    /**
     * @brief Get the parameter value for number of constraint points per piece.
     * This is user-defined
     * 
     * @return int 
     */
    int get_cps_num_perPiece_(void) { return cps_num_perPiece_; }

    /**
     * @brief Get the user-defined swarm clearance parameter
     * 
     * @return double 
     */
    double get_swarm_clearance_(void) { return swarm_clearance_; }

    /* main planning API */

    /**
     * @brief Optimize a trajectory given boundary conditions, inner points and segment durations.
     * 
     * 
     * @param iniState Initial state
     * @param finState Final state
     * @param initInnerPts Inner points
     * @param initT Time duration at each point
     * @param init_cstr_pts Initial constraint points
     * @param final_cost 
     * @return true 
     * @return false 
     */
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            Eigen::MatrixXd &optimal_points, double &final_cost);

    /**
     * @brief Get points along trajectory to check
     * 
     * @param traj Trajectory to check
     * @param num_pieces number of pieces to check
     * @param pts_check Vector of points to check
     * @return true 
     * @return false 
     */
    bool computePointsToCheck(poly_traj::Trajectory &traj, int num_pieces, PtsChk_t &pts_check);

    std::vector<std::pair<int, int>> finelyCheckConstraintPointsOnly(Eigen::MatrixXd &init_points);

    /**
     * @brief Check for collision along path and set {p,v} pairs to constraint points
     * 
     */
    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              const poly_traj::MinJerkOpt &pt_data,
                                              const bool flag_first_init /*= true*/);

    bool roughlyCheckConstraintPoints(void);

    /* multi-topo support */
    std::vector<ConstraintPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);

  private:
    /**
     * @brief The LBFGS callback function to provide function and gradient evaluations given a current values of variables
     * 
     * @param func_data The user data sent for lbfgs_optimize() function by the client.
     * @param x         The current values of variables.
     * @param grad      The gradient vector. The callback function must compute
     *                      the gradient values for the current variables.
     * @param n         The number of variables.
     * @return double   The value of the objective function for the current
     *                          variables.
     */
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);

    /**
     * @brief The LBFGS callback function to receive the progress (the number of iterations, the current value of the objective function) of the minimization process.
     * 
     * @param func_data 
     * @param x 
     * @param g 
     * @param fx 
     * @param xnorm 
     * @param gnorm 
     * @param step 
     * @param n 
     * @param k 
     * @param ls 
     * @return int 
     */
    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    /**
     * @brief Gradient 
     * 
     * @tparam EIGENVEC 
     * @param gdT Gradient of size of number of pieces
     * @param costs a vector of costs
     * @param K Constraint points per piece, or total sample number
     */
    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector3d &p,
                           Eigen::Vector3d &gradp,
                           double &costp);

    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    /**
     * @brief Set the cost and gradient based on given velocity
     * 
     * @param v Vector of (x,y,z) velocities
     * @param gradv Gradient
     * @param costv cost 
     * @return true 
     * @return false 
     */
    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    void lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                      const int n,
                                      Eigen::MatrixXd &gdp,
                                      double &var);

    bool formationGradCostP(const int i_dp,
                            const double t,
                            const Eigen::Vector3d &p,
                            const Eigen::Vector3d &v,
                            Eigen::Vector3d &gradp,
                            double &gradt,
                            double &grad_prev_t,
                            double &costp);

  public:
    typedef std::unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace ego_planner
#endif //_POLY_TRAJ_OPTIMIZER_H_