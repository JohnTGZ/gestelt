#ifndef _POLYHEDRON_SFC_OPTIMIZER_H_
#define _POLYHEDRON_SFC_OPTIMIZER_H_

#include <algorithm>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <grid_map/grid_map.h>
#include <path_searching/dyn_a_star.h>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/planning_visualization.h>
#include "optimizer/lbfgs.hpp"
#include "poly_traj_utils.hpp"

namespace back_end
{

  class PolyhedronSFCOptimizer
  {
  public:
    PolyhedronSFCOptimizer(){}
    ~PolyhedronSFCOptimizer() {}

    void reset();

    void setParam(ros::NodeHandle &pnh);

    /* main planning API */

    bool genInitialSFCTrajectory(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &end_pos,
      const std::vector<Eigen::Matrix3Xd> &vPolys, 
      const double &smoothD,
      Eigen::Matrix3Xd &path);

    /**
     * @brief 
     * 
     * @param startPVA        Starting position, velocity, acceleration
     * @param endPVA          Final position, velocity, acceleration
     * @param inner_ctrl_pts  Initial inner control points
     * @param init_seg_dur    Initial segment durations
     * @param vPolyIdx 
     * @param vPolytopes 
     * @param num_decis_var_t     Number of decision variables (segment time durations)
     * @param num_decis_var_bary  Number of decision variables (barycentric weights)
     * @param final_cost 
     * @return true 
     * @return false 
     */
    bool optimizeTrajectory(const Eigen::Matrix3d &startPVA, const Eigen::Matrix3d &endPVA,
                            const Eigen::MatrixXd &inner_ctrl_pts, const Eigen::VectorXd &init_seg_dur,
                            const Eigen::VectorXi& vPolyIdx, const std::vector<Eigen::Matrix3Xd>& vPolytopes,
                            const int& num_decis_var_t, const int& num_decis_var_bary,
                            double &final_cost);

  private:
    /* Optimizer callbacks */

    /**
     * @brief The LBFGS callback function to provide function and gradient evaluations given a current values of variables
     * 
     * @param func_data The user data sent for lbfgs_optimize() function by the client.
     * @param x         The current values of variables.
     * @param grad      The gradient vector. The callback function must compute the gradient values for the current variables.
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

    /**
     * @brief Get cost for constraints on PVA
     * 
     * @tparam EIGENVEC 
     * @param gdT Gradient of size of number of pieces
     * @param costs a vector of costs
     * @param K Constraint points per piece, or total sample number
     */
    template <typename EIGENVEC>
    void addPVAGradCost2CT(
      EIGENVEC &gdT, Eigen::VectorXd &obs_swarm_feas_qvar_costs, const int &K, poly_traj::MinJerkOpt& mjo);

    /**
     * @brief Cost of swarm 
     * 
     * @param t Current time
     * @param p position
     * @param v velocity
     * @param gradp 
     * @param gradt 
     * @param grad_prev_t 
     * @param costp 
     * @return true 
     * @return false 
     */
    bool swarmGradCostP(const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    /**
     * @brief Penalty on variance of distance between each point i.e. penalize the non-uniformity of distance between points
     * 
     * @param ps constraint points 
     * @param gdp 
     * @param var 
     */
    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    // Get distance cost for calculating cost of trajectory
    static double distanceCost( void *func_data,
                                const double* xi,
                                double* gradXi,
                                const int n);

    // Get virtual time gradient cost
    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /**
     * @brief 
     * 
     * @param x 
     * @param mu 
     * @param f 
     * @param df 
     * @return true if cost is more than zero 
     * @return false 
     */
    static inline bool smoothedL1(const double &x,
                                  const double &mu,
                                  double &f,
                                  double &df)
    {
        if (x <= 0.0)
        {
          return false;
        }
        else if (x > mu)
        {
          f = x - 0.5 * mu;
          df = 1.0;
          return true;
        }
        else
        {
          const double xdmu = x / mu;
          const double mumxd2 = mu - 0.5 * x;
          f = mumxd2 * pow(xdmu,3);
          df = pow(xdmu,2) * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
          return true;
        }
    }


  public: 

    /* getters */

    /**
     * @brief Get the user-defined value for number of constraint points per segment.
     * 
     * @return int 
     */
    int getNumCstrPtsPerSeg(void) const { 
      return cps_num_perPiece_; 
    }

    /**
     * @brief Get the user-defined value for number of constraint points per segment.
     * 
     * @return int 
     */
    int getMaxVel(void) const { 
      return max_vel_; 
    }

    /* setters */

    // Set the map
    void setEnvironment(const std::shared_ptr<GridMap> &map)
    {
      grid_map_ = map;
    }

    // Set the visualizer
    void setVisualizer(ego_planner::PlanningVisualization::Ptr vis)
    {
      visualization_ = vis;
    }

    // Set the pointer to the swarm trajectories
    void assignSwarmTrajs(
      std::shared_ptr<std::vector<ego_planner::LocalTrajData>> swarm_local_trajs) {
      swarm_local_trajs_ = swarm_local_trajs;
    }

  /* Constraint elimination methods */
  public:

    /**
     * @brief Convert from real to virtual time
     * 
     * @tparam EIGENVEC 
     * @param RT 
     * @param VT 
     */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
    {
      for (int i = 0; i < RT.size(); ++i)
      {
        VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                            : (1.0 - sqrt(2.0 / RT(i) - 1.0));
      }
    }

    /**
     * @brief Convert from virtual to real time
     * 
     * @tparam EIGENVEC 
     * @param RT 
     * @param VT 
     */
    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
    {
      for (int i = 0; i < VT.size(); ++i)
      {
        RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                            : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
      }
    }

    /**
     * @brief Convert from unconstrained xi coordinates into q constrained coordinates
     * 
     * @param xi 
     * @param vIdx 
     * @param vPolys 
     * @param P 
     */
    static inline void forwardP(const Eigen::VectorXd &xi,
                                const Eigen::VectorXi &vIdx,
                                const PolyhedraV &vPolys,
                                Eigen::Matrix3Xd &P)
    {
        const int sizeP = vIdx.size();
        P.resize(3, sizeP);
        Eigen::VectorXd q;
        for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
        {
            l = vIdx(i);
            k = vPolys[l].cols();
            q = xi.segment(j, k).normalized().head(k - 1);
            P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                        vPolys[l].col(0);
        }
        return;
    }

    /**
     * @brief Convert from unconstrained xi coordinates into q constrained coordinates.
     * This is done by minimizing the squared distance between f_H(xi) and P.
     *  
     * @tparam EIGENVEC 
     * @param P 
     * @param vIdx 
     * @param vPolys 
     * @param xi 
     */
    template <typename EIGENVEC>
    static inline void backwardP(const Eigen::Matrix3Xd &P,
                                  const Eigen::VectorXi &vIdx,
                                  const std::vector<Eigen::Matrix3Xd> &vPolys,
                                  EIGENVEC &xi)
    {
        double final_cost; // final cost in minimum squared distance

        lbfgs::lbfgs_parameter_t lbfgs_params; // least squares
        lbfgs_params.past = 0;
        lbfgs_params.delta = 1.0e-5;
        lbfgs_params.g_epsilon = FLT_EPSILON;
        lbfgs_params.max_iterations = 128;

        Eigen::Matrix3Xd ovPoly;
        for (int i = 0, j = 0, k, l; i < P.cols(); i++, j += k) // For each inner point
        {
            l = vIdx(i);
            k = vPolys[l].cols();

            ovPoly.resize(3, k + 1);
            ovPoly.col(0) = P.col(i);
            ovPoly.rightCols(k) = vPolys[l];

            double x[k]; // Total number of vertices for each "overlap" polyhedron
            std::fill(x, x + k, sqrt(1.0 / k));

            int num_decis_var = k;
            lbfgs::lbfgs_optimize(
                                  num_decis_var,
                                  x,
                                  &final_cost,
                                  PolyhedronSFCOptimizer::costTinyNLS,
                                  NULL,
                                  NULL,
                                  &ovPoly,
                                  &lbfgs_params);

            xi.segment(j, k) = Eigen::Map<Eigen::VectorXd>(x+j, k);
            // xi.block<k, 1>(j, 0) = 
            // xi.segment(j, k) = x;
        }

        return;
    }

    static inline double costTinyNLS( void *func_data,
                                      const double* xi_arr,
                                      double* gradXi_arr, 
                                      const int n)
    {
        const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)func_data;

        // TODO: convert xi from array to vectorxd
        Eigen::Map<const Eigen::VectorXd> xi(xi_arr, n);

        const double sqrNormXi = xi.squaredNorm();
        const double invNormXi = 1.0 / sqrt(sqrNormXi);

        const Eigen::VectorXd unitXi = xi * invNormXi;
        const Eigen::VectorXd r = unitXi.head(n - 1);
        const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                      ovPoly.col(1) - ovPoly.col(0);

        double cost = delta.squaredNorm();

        // TODO: convert gradXi from array to vectorxd
        Eigen::Map<Eigen::VectorXd> gradXi(gradXi_arr, n);

        gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                              r.array() * 2.0;
        gradXi(n - 1) = 0.0;
        gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

        const double sqrNormViolation = sqrNormXi - 1.0;
        if (sqrNormViolation > 0.0)
        {
            double c = sqrNormViolation * sqrNormViolation;
            const double dc = 3.0 * c;
            c *= sqrNormViolation;
            cost += c;
            gradXi += dc * 2.0 * xi;
        }

        return cost;
    }

    template <typename EIGENVEC>
    static inline void backwardGradT(const Eigen::VectorXd &tau,
                                      const Eigen::VectorXd &gradT,
                                      EIGENVEC &gradTau)
    {
        const int sizeTau = tau.size();
        gradTau.resize(sizeTau);
        double denSqrt;
        for (int i = 0; i < sizeTau; i++)
        {
            if (tau(i) > 0)
            {
                gradTau(i) = gradT(i) * (tau(i) + 1.0);
            }
            else
            {
                denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
            }
        }

        return;
    }

  /* private class members */
  public:
    typedef std::unique_ptr<PolyhedronSFCOptimizer> Ptr;

    /* Data structures */
    std::vector<Eigen::MatrixXd> intermediate_cstr_pts_xi_; // Intermediate constraint points unconstrained xi coordinates
    std::vector<Eigen::MatrixXd> intermediate_cstr_pts_q_; // Intermediate constraint points constrained q coordinates
      
  private:
    std::shared_ptr<GridMap> grid_map_;         // Occupancy map 
    ego_planner::PlanningVisualization::Ptr visualization_; // visualizer
    std::shared_ptr<std::vector<ego_planner::LocalTrajData>> swarm_local_trajs_; // Swarm MINCO trajectories

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    int drone_id_;            // ID of drone
    int num_segs_;          // poly traj piece numbers
    int iter_num_;           // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in swarm

    /* optimization parameters */
    int cps_num_perPiece_;   // number of distinctive constraint points per piece

    double weight_poly_bounds_;                                   // Polyhedron bounds weight
    double wei_swarm_;                                            // swarm weight
    double wei_feas_;                                             // feasibility weight
    double wei_sqrvar_;                                           // squared variance weight
    double wei_time_;                                             // time weight
    double obs_clearance_, swarm_clearance_;                      // safe distance
    double max_vel_, max_acc_;                                    // dynamic limits

    double t_now_;

    /* Data structures */
    poly_traj::MinJerkOpt mjo_q_;   // Minimum jerk trajectory in q space
    poly_traj::MinJerkOpt mjo_xi_;  // Minimum jerk trajectory in xi space

    Eigen::MatrixXd cstr_pts_xi_; // inner CONSTRAINT points of trajectory (excludes boundary points), this is finer than the inner CONTROL points
    Eigen::MatrixXd cstr_pts_q_; // inner CONSTRAINT points of trajectory (excludes boundary points), this is finer than the inner CONTROL points

    int num_decis_var_bary_, num_decis_var_t_; // Number of decision variables for barycentric weights and segment time durations

    Eigen::VectorXi vPolyIdx_;
    std::vector<Eigen::Matrix3Xd> vPolytopes_;
    Eigen::MatrixXd inner_ctrl_pts_;
    Eigen::VectorXd time_seg_durs_;

  }; // class PolyhedronSFCOptimizer

} // namespace back_end

#endif //_POLYHEDRON_SFC_OPTIMIZER_H_