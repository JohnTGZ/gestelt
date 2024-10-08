#include "optimizer/polyhedron_sfc_optimizer.h"

namespace back_end
{
  bool PolyhedronSFCOptimizer::genInitialSFCTrajectory(
    const Eigen::Vector3d &start_pos, const Eigen::Vector3d &end_pos,
    const std::vector<Eigen::Matrix3Xd> &vPolys, 
    const double &smoothD,
    Eigen::Matrix3Xd &path)
  {
    
    if (vPolys.size() == 1){
      path.resize(3, 2);
      path.leftCols<1>() = start_pos;
      path.rightCols<1>() = end_pos;
      return true;
    }

    // overlaps: division by 2 is arbitrary value
    const int overlaps = vPolys.size() / 2; 

    Eigen::VectorXi vSizes(overlaps); // Number of overlaps
    for (int i = 0; i < overlaps; i++)
    {
      // Each vPolys contains a 3*M matrix. The number of columns = number of vertices
      vSizes(i) = vPolys[2 * i + 1].cols(); // Store Number of vertices of each "overlap" polyhedron
    }

    // xi: barycentric coordinates
    double xi[vSizes.sum()]; // Total number of vertices for each "overlap" polyhedron
    // Eigen::VectorXd xi(vSizes.sum()); 
    for (int i = 0, j = 0; i < overlaps; i++)
    {
      // For block containing number of vertices starting at j
      //      set all weights at 1/(num vertices)
      std::fill(xi+j, xi+j+vSizes(i), sqrt(1.0 / vSizes(i)));

      j += vSizes(i);
    }

    int num_decis_var = vSizes.sum();

    double final_cost; // To store total cost
    void *dataPtrs[4];
    dataPtrs[0] = (void *)(&smoothD); // smoothingFactor
    dataPtrs[1] = (void *)(&start_pos);
    dataPtrs[2] = (void *)(&end_pos);
    dataPtrs[3] = (void *)(&vPolys);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.past = 3;
    lbfgs_params.delta = init_traj_delta_;
    lbfgs_params.g_epsilon = init_traj_g_eps_;
    // lbfgs_params.max_linesearch = 200;
    // lbfgs_params.min_step = 1e-32;
    // lbfgs_params.max_step = 1e+20;

    int result = lbfgs::lbfgs_optimize(
        num_decis_var,                       // number of variabless
        xi,                                 // x: decision variables
        &final_cost,                        // f
        PolyhedronSFCOptimizer::distanceCost, // proc_evaluate
        NULL,                            // proc_stepbound
        NULL,                            // proc_progress
        dataPtrs,                        // User data for the client program. The callback functions will receive the value of this argument.
        &lbfgs_params);                  // param

    // IF converged, maxmimum_iteration, already minimized or stop
    if (result == lbfgs::LBFGS_CONVERGENCE ||
        result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
        result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
        result == lbfgs::LBFGS_STOP)
    {
      // continue onwards to construct path
    }
    // ELSE IF Cancelled
    else if (result == lbfgs::LBFGSERR_CANCELED)
    {
      std::cout << "[PolyhedronSFCOptimizer::genInitialSFCTrajectory] Optimization Cancelled!" << std::endl;
      return false;
    }
    // ELSE ERROR
    else
    {
      ROS_WARN("[PolyhedronSFCOptimizer::genInitialSFCTrajectory] UAV %d: Solver error. Return = %d, %s. Skip this planning.", drone_id_, result, lbfgs::lbfgs_strerror(result));
      return false;
    }

    path.resize(3, overlaps + 2);
    path.leftCols<1>() = start_pos;
    path.rightCols<1>() = end_pos;
    Eigen::VectorXd r;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
    {
      // k: number of vertices of "overlap" polyhedron
      k = vPolys[2 * i + 1].cols(); 
      // q: Get weights (barycentric) corresponding to the "overlap" polyhedron
      Eigen::Map<const Eigen::VectorXd> q(xi + j, k);

      r = q.normalized().head(k - 1); // get Block containing (k - 1) weights 
      
      // Get coordinates of the path
      path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
    }

    return true;
  }

  void PolyhedronSFCOptimizer::reset(){
    // reset all existing variables
  }

  bool PolyhedronSFCOptimizer::optimizeTrajectory(
      const Eigen::Matrix3d &startPVA, const Eigen::Matrix3d &endPVA,
      const Eigen::MatrixXd &P, const Eigen::VectorXd &init_seg_dur,
      const Eigen::VectorXi& vPolyIdx, const std::vector<Eigen::Matrix3Xd>& vPolytopes,
      const Eigen::VectorXi& hPolyIdx, const std::vector<Eigen::MatrixX4d>& hPolytopes,
      const int& num_decis_var_t, const int& num_decis_var_bary,
      double &final_cost)
  {

    // IF size of inner points and segment durations are not the same, there is a bug
    if (P.cols() != (init_seg_dur.size() - 1))
    {
      ROS_ERROR("[PolyhedronSFCOptimizer::optimizeTrajectory] P.cols() != (init_seg_dur.size()-1)");
      return false;
    }

    reset();

    int restart_nums = 0, num_retries = 0;
    bool flag_force_return{false}, flag_success{false};

    num_segs_ = init_seg_dur.size(); // Number of segments
    int num_decis_var = num_decis_var_t + num_decis_var_bary; //  Number of decision variables. Position (size 3 x (M-1)) + Time ( size 1 x (M-1) + 1) 
    
    num_decis_var_t_ = num_decis_var_t;
    num_decis_var_bary_ = num_decis_var_bary;
    
    vPolyIdx_ = vPolyIdx;
    vPolytopes_ = vPolytopes;
    hPolyIdx_ = hPolyIdx;
    hPolytopes_ = hPolytopes;

    P_ = P;             // assign initial inner control points
    T_ = init_seg_dur;  // assign initial segment time durations
    grad_P_.resize(3, num_decis_var_t_ - 1); // resize gradient of inner control points

    /* Initialize decision variables*/
    double x_init[num_decis_var]; // Initial decision variables: Array of [ P, init_seg_dur ]
    std::fill(x_init, x_init + num_decis_var, 0.0); 

    Eigen::Map<Eigen::VectorXd> tau(x_init, num_decis_var_t_); 
    Eigen::Map<Eigen::VectorXd> xi(x_init + num_decis_var_t_, num_decis_var_bary_); 

    // Convert from real to virtual time tau
    RealT2VirtualT(init_seg_dur, tau);
    // // Convert from constrained coordinates q to barycentric coordinates xi
    backwardP(P_, vPolyIdx_, vPolytopes_, xi);

    /* Initialize minimum jerk trajectory */
    mjo_q_.reset(startPVA, endPVA, num_segs_);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 256; 
    lbfgs_params.max_iterations = 200;
    lbfgs_params.g_epsilon = final_traj_g_eps_;
    lbfgs_params.delta = final_traj_delta_;
    lbfgs_params.past = 3;
    // lbfgs_params.max_linesearch = 200;
    lbfgs_params.min_step = 1e-32;
    // lbfgs_params.max_step = 1e+20;

    do
    {
      /* ---------- prepare ---------- */
      iter_num_ = 0;
      flag_force_return = false;
      flag_success = false;
      t_now_ = ros::Time::now().toSec();

      /* ---------- optimize ---------- */
      double t1 = ros::Time::now().toSec();
      int result = lbfgs::lbfgs_optimize(
          num_decis_var,                  // The number of variables 
          x_init,                         // The array of variables.
          &final_cost,                    // The pointer to the variable that receives the final value of the objective function for the variables
          PolyhedronSFCOptimizer::costFunctionCallback, // The callback function to provide function and gradient evaluations given a current values of variables
          NULL,                           //  The callback function to provide values of the upperbound of the stepsize to search in, provided with the beginning values of variables before the linear search, and the current step vector (can be negative gradient).
          NULL,                           // The callback function to receive the progress (the number of iterations, the current value of the objective function) of the minimization process.
          this,                           // A user data for the client program. The callback functions will receive the value of this argument.
          &lbfgs_params);                 // The pointer to a structure representing parameters for L-BFGS optimization. A client program can set this parameter to NULL to use the default parameters
      double opt_time_ms = (ros::Time::now().toSec() - t1)* 1000;

      // IF converged, maxmimum_iteration, already minimized or stop
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        flag_force_return = false;
        // TODO: Add collision-checking in the path
        // printf("\033[32m [PolyhedronSFCOptimizer::optimizeTrajectory]: UAV %d: Optimization Succeeded! iter=%d, time(ms)=%5.3f, cost=%5.3f\n \033[0m", drone_id_, iter_num_, opt_time_ms, final_cost);
        flag_success = true;
      }
      // ELSE IF Cancelled
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        num_retries++; 
        std::cout << "[PolyhedronSFCOptimizer::optimizeTrajectory] Optimization Cancelled!" << std::endl;
        // std::cout << "iter=" << iter_num_ << ",time(ms)=" << opt_time_ms << ",rebound." <<std::endl;
      }
      // ELSE ERROR
      else
      {
        std::cout << "iter=" << iter_num_ << ",time(ms)=" << opt_time_ms << ",error." <<std::endl;
        ROS_WARN("[PolyhedronSFCOptimizer::optimizeTrajectory] UAV %d: Solver error. Return = %d, %s. Skip this planning.", drone_id_, result, lbfgs::lbfgs_strerror(result));
      }

    } while ((flag_force_return && num_retries <= 20));

    return flag_success;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyhedronSFCOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {

    // Pointer to current instance of PolyhedronSFCOptimizer
    PolyhedronSFCOptimizer *opt = reinterpret_cast<PolyhedronSFCOptimizer *>(func_data);

    const int dimTau = opt->num_decis_var_t_;
    const int dimXi =  opt->num_decis_var_bary_;

    Eigen::Map<const Eigen::VectorXd> tau(x, dimTau);         // virtual time
    Eigen::Map<const Eigen::VectorXd> xi(x + dimTau, dimXi);  // barycentric weights

    // Convert from virtual to real time
    opt->VirtualT2RealT(tau, opt->T_);
    // Convert from xi into q (P_)
    opt->forwardP(xi, opt->vPolyIdx_, opt->vPolytopes_, opt->P_);

    // Gradients 
    Eigen::Map<Eigen::VectorXd> grad_tau(grad, dimTau);         // gradient of virtual time
    Eigen::Map<Eigen::VectorXd> grad_xi(grad + dimTau, dimXi);  // gradient of xi

    Eigen::VectorXd grad_T(opt->num_decis_var_t_);  // p.d.(H / t_i)

    // Initialize cost values
    double jerk_cost = 0, time_cost = 0;
    opt->min_ellip_dist2_ = std::numeric_limits<double>::max();

    // MJO in constrained q coordinates
    opt->mjo_q_.generate(opt->P_, opt->T_); // Generate minimum jerk trajectory
    opt->cstr_pts_q_ = opt->mjo_q_.getInitConstraintPoints(opt->cps_num_perPiece_); // Discretize trajectory into inner constraint points

    /** 1. Jerk cost 
     * jerk_cost is trajectory jerk cost
     */
    // Get gradient of real time and jerk cost
    opt->mjo_q_.initGradCost(grad_T, jerk_cost); // In initGradCost(...), do addGradJbyT(gdT) and addGradJbyC(gdC);

    /** 2. Time integral cost 
      *   2a. Static obstacle cost
      *   2b. Swarm cost
      *   2c. Dynamical feasibility
      *   2d. Uniformity of points
    */
    Eigen::VectorXd obs_swarm_feas_qvar_costs(4); // Vector of costs containing (Static obstacles, swarm, dynamic, feasibility, qvar)
    opt->addPVAGradCost2CT(grad_T, obs_swarm_feas_qvar_costs, 
                            opt->cps_num_perPiece_, opt->mjo_q_,
                            opt->hPolyIdx_, opt->hPolytopes_ ); 

    /** 3. Update the gradient costs p.d.(H / T_i) and p.d.(H / xi)
    */
    opt->mjo_q_.getGrad2TP(grad_T, opt->grad_P_); // Update the p.d.(H/T_i) and p.d.(H/q) 

    /** 3. Get gradients 
    */
    // Get gradient of virtual time (grad_tau)
    opt->VirtualTGradCost(opt->T_, tau, grad_T, grad_tau, time_cost);

    // Get gradient w.r.t xi (grad_xi)
    backwardGradP(xi, opt->vPolyIdx_, opt->vPolytopes_, opt->grad_P_, grad_xi);
    normRetrictionLayer(xi, opt->vPolyIdx_, opt->vPolytopes_, jerk_cost, grad_xi);

    opt->iter_num_++;

    return jerk_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
  }


  /* Cost functions */

  void PolyhedronSFCOptimizer::setParam(ros::NodeHandle &pnh)
  {
    pnh.param("drone_id", drone_id_, -1);

    pnh.param("optimization/num_cstr_pts_per_seg", cps_num_perPiece_, -1);
    pnh.param("optimization/weight_poly_bounds", weight_poly_bounds_, -1.0);
    pnh.param("optimization/weight_swarm", wei_swarm_, -1.0);
    pnh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    pnh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    pnh.param("optimization/weight_time", wei_time_, -1.0);
    pnh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
    pnh.param("optimization/max_vel", max_vel_, -1.0);
    pnh.param("optimization/max_acc", max_acc_, -1.0);

    pnh.param("optimization/poly/init_traj_g_eps", init_traj_g_eps_, 0.01);
    pnh.param("optimization/poly/init_traj_delta", init_traj_delta_, 0.01);
    pnh.param("optimization/poly/backward_proj_g_eps", backward_proj_g_eps_, 0.01);
    pnh.param("optimization/poly/backward_proj_delta", backward_proj_delta_, 0.01);
    pnh.param("optimization/poly/final_traj_g_eps", final_traj_g_eps_, 0.01);
    pnh.param("optimization/poly/final_traj_delta", final_traj_delta_, 0.01);

  }



  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyhedronSFCOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &T, const EIGENVEC &tau,
      const Eigen::VectorXd &grad_T, EIGENVECGD &grad_tau,
      double &costT)
  {

    for (int i = 0; i < tau.size(); ++i)
    {
      double gdVT2Rt;
      if (tau(i) > 0)
      {
        gdVT2Rt = tau(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
        gdVT2Rt = (1.0 - tau(i)) / (denSqrt * denSqrt);
      }

      grad_tau(i) = (grad_T(i) + wei_time_) * gdVT2Rt;
    }

    costT = T.sum() * wei_time_;
  }

  template <typename EIGENVEC>
  void PolyhedronSFCOptimizer::addPVAGradCost2CT(
      EIGENVEC &gdT, Eigen::VectorXd &obs_swarm_feas_qvar_costs, 
      const int &K, poly_traj::MinJerkOpt& mjo,
      const Eigen::VectorXi& hPolyIdx, const std::vector<Eigen::MatrixX4d>& hPolytopes)
  {
    int N = num_segs_; // N: Number of segments
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradp, gradv, grada; // Each Gradient is a vector with (x,y,z) components
    double costp; 
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3; // Time basis
    double s1, s2, s3, s4, s5;    // time (s1 = t, s2 = t^2, ..., s5 = t^5)
    double step, alpha;           // step: time step, alpha: 
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omega;    // Quadrature coefficient
    int idx_cp = 0; // Index of constraint point
    obs_swarm_feas_qvar_costs.setZero();

    double t = 0;
    for (int i = 0; i < N; ++i) // for each piece/segment number
    {
      // c: Polynomial coefficients 
      const Eigen::Matrix<double, 6, 3> &c = mjo.get_b().block<6, 3>(i * 6, 0); 
      double T_i = mjo.getSegDurations()(i);
      step = T_i / K; // Duration of each piece / sample number.
      s1 = 0.0; // Time t, it will increase with each step of the constraint point

      for (int j = 0; j <= K; ++j) // For each constraint point (or sample) in the segment. This is also known as the sample index
      {
        s2 = s1 * s1;   // t^2
        s3 = s2 * s1;   // t^3
        s4 = s2 * s2;   // t^4
        s5 = s4 * s1;   // t^5
        // betax: Time bases 
        beta0 << 1.0, s1, s2, s3, s4, s5;                           // (1, t, t^2,  t^3,  t^4,    t^5)
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;  // (0, 1, 2t,   3t^2, 4t^3,   5t^4)
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;     // (0, 0, 1,    6t,   12t^2,  20t^3)
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;          // (0, 0, 0,    1,    24t,    60t^2)
        alpha = j / K ; // progress along segment/piece, value is from 0 to 1

        // p_i(t) = c_i.T * beta(t) // (m, 1) = (m, 2s) * (2s, 1)
        pos = c.transpose() * beta0; // position at current constraint point
        vel = c.transpose() * beta1; // velocity at current constraint point
        acc = c.transpose() * beta2; // acceleration at current constraint point
        jer = c.transpose() * beta3; // jerk at current constraint point

        // omega: quadrature coefficients using trapezoid rule
        omega = (j == 0 || j == K) ? 0.5 : 1.0;

        /**
         * Penalty on exceeding bounds of polyhedrons
         */
        int L = hPolyIdx(i);          // Get the polytope index belonging to segment i
        int num_hyp_planes = hPolytopes[L].rows(); // Get the rows(hyperplanes) of the L-th polyhedron
        for (int k = 0; k < num_hyp_planes; k++) // For each hyperplane in L-th polyhedron
        {
            Eigen::Vector3d outerNormal = hPolytopes[L].block<1, 3>(k, 0);
            double poly_pen = outerNormal.dot(pos) + hPolytopes[L](k, 3); // polyhedron penalty

            if (poly_pen > 0){
              Eigen::Matrix<double, 6, 3> pd_constr_c_i = beta0 * outerNormal.transpose();     // Partial derivative of constraint w.r.t c_i // (2s, m) = (2s, 1) * (1, m)
              double pd_constr_t = (outerNormal).dot(vel);// P.D. of constraint w.r.t t        // (1,1) = (1, 2s) * (2s, m) * (m, 1)

              // Intermediate calculations for chain rule to get partial derivatives of cost J
              double pd_cost_constr = 3 * (T_i / K) * omega * weight_poly_bounds_ * pow(poly_pen, 2) ; // P.D. of cost J w.r.t constraint
              double cost = (T_i / K) * omega * weight_poly_bounds_ * pow(poly_pen, 3);              
              double pd_t_T_i = (j / K);                                                                // P.D. of time t w.r.t T_i

              // 1) Partial derivatives of Cost J
              //    1a) w.r.t coefficients c_i
              Eigen::Matrix<double, 6, 3> pd_cost_c_i = pd_cost_constr * pd_constr_c_i; // P.D. of cost J w.r.t c_i. Uses chain rule // (m,2s)

              //    1b) w.r.t time t
              double pd_cost_t = cost / T_i  + pd_cost_constr * pd_constr_t * pd_t_T_i;// P.D. of cost w.r.t t // (1,1)
              // ANNOTATION: (EXPANDED) pd_cost_t = ((T_i / K) * weight_poly_bounds_ * omega *  pow(poly_pen, 3)) / T_i  + (3 * (T_i / K) * omega * weight_poly_bounds_ * pow(poly_pen, 2)) * pd_constr_t * (j / K)

              // Sum up sampled costs
              mjo.get_gdC().block<6, 3>(i * 6, 0) += pd_cost_c_i; // Gradient of cost J w.r.t polynomial coefficients
              // ANNOTATION: (EXPANDED) mjo.get_gdC().block<6, 3>(i * 6, 0) += 3 * (T_i / K) * omega * weight_poly_bounds_ * pow(poly_pen, 2) * pd_constr_c_i
              gdT(i) += pd_cost_t; 

              obs_swarm_feas_qvar_costs(0) += cost;
            }
        }

        // /**
        //  * Penalty on clearance to swarm/dynamic obstacles
        //  */
        double gradt, grad_prev_t;
        if (swarmGradCostP(t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt;
          // Sum up sampled costs
          mjo.get_gdC().block<6, 3>(i * 6, 0) += omega * step * gradViolaPc;
          gdT(i) += omega * (costp / K + step * gradViolaPt); // Add to gradient for i-th segment
          if (i > 0) // If not the first segment
          {
            gdT.head(i).array() += omega * step * grad_prev_t;
          }
          obs_swarm_feas_qvar_costs(1) += omega * step * costp;
        }
        
        /**
         * Penalty on velocity constraint, vector of (x,y,z)
         */
        double vel_pen = vel.squaredNorm() - max_vel_ * max_vel_; 
        if (vel_pen > 0)
        {  
          // Partial derivatives of Constraint
          Eigen::Matrix<double, 6, 3> pd_constr_c_i = 2 * beta1 * vel.transpose(); // Partial derivative of constraint w.r.t c_i // (2s, m) = (2s, 1) * (1, m)
          double pd_constr_t =  2 * beta2.transpose() * c * vel;// P.D. of constraint w.r.t t // (1,1) = (1, 2s) * (2s, m) * (m, 1)

          // Intermediate calculations for chain rule to get partial derivatives of cost J
          double pd_cost_constr = 3 * (T_i / K) * omega * wei_feas_ * pow(vel_pen,2) ; // P.D. of cost w.r.t constraint
          double cost = (T_i / K) * omega * wei_feas_ * pow(vel_pen,3); 
          double pd_t_T_i = (j / K); // P.D. of time t w.r.t T_i

          // Partial derivatives of Cost J
          //    w.r.t coefficients c_i
          Eigen::Matrix<double, 6, 3> pd_cost_c_i = pd_cost_constr * pd_constr_c_i; // P.D. of cost w.r.t c_i. Uses chain rule // (m,2s)
          //    w.r.t time t
          double pd_cost_t = cost / T_i  + pd_cost_constr * pd_constr_t * pd_t_T_i;// P.D. of cost w.r.t t // (1,1)

          // Sum up sampled costs
          mjo.get_gdC().block<6, 3>(i * 6, 0) += pd_cost_c_i; // Gradient of cost w.r.t polynomial coefficients
          gdT(i) += pd_cost_t; 
          obs_swarm_feas_qvar_costs(2) += cost; 
        }

        /**
         * Penalty on acceleration constraint, vector of (x,y,z)
         */
        double acc_pen = acc.squaredNorm() - max_acc_ * max_acc_; 
        if (acc_pen > 0)
        {  
          // Partial derivatives of Constraint
          Eigen::Matrix<double, 6, 3> pd_constr_c_i = 2 * beta2 * acc.transpose(); // Partial derivative of constraint w.r.t c_i // (2s, m) = (2s, 1) * (1, m)
          double pd_constr_t =  2 * beta3.transpose() * c * acc;// P.D. of constraint w.r.t t // (1,1) = (1, 2s) * (2s, m) * (m, 1)

          // Intermediate calculations for chain rule to get partial derivatives of cost J
          double pd_cost_constr = 3 * (T_i / K) * omega * wei_feas_ * pow(acc_pen,2) ; // P.D. of cost w.r.t constraint
          double cost = (T_i / K) * omega * wei_feas_ * pow(acc_pen,3);
          double pd_t_T_i = (j / K); // P.D. of time t w.r.t T_i

          // Partial derivatives of Cost J 
          //    w.r.t coefficients c_i
          Eigen::Matrix<double, 6, 3> pd_cost_c_i = pd_cost_constr * pd_constr_c_i; // P.D. of cost w.r.t c_i. Uses chain rule // (m,2s)
          //    w.r.t time t
          double pd_cost_t = cost / T_i  + pd_cost_constr * pd_constr_t * pd_t_T_i;// P.D. of cost w.r.t t // (1,1)

          // Sum up sampled costs
          mjo.get_gdC().block<6, 3>(i * 6, 0) += pd_cost_c_i; // Gradient of cost w.r.t polynomial coefficients
          gdT(i) += pd_cost_t; 
          obs_swarm_feas_qvar_costs(2) += cost; 
        }

        // printf("L\n");
        s1 += step; // Step to next time step

        // If current CP is not last in the segment
        // OR current CP is last point of last segment
        if (j != K || (j == K && i == N - 1))
        {
          ++idx_cp;
        }
      } // end iteration through all constraint points in segment

      t += mjo.getSegDurations()(i);
    } // end iteration through all segments

    /**
     * Penalty on quadratic variance
     */
    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cstr_pts_q_, gdp, var);

    // What cost is this?
    idx_cp = 0;
    for (int i = 0; i < N; ++i) // for each piece/segment number
    {
      step = mjo.getSegDurations()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j) // For each constraint point
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = mjo.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omega = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(idx_cp).transpose();
        gradViolaPt = alpha * gdp.col(idx_cp).transpose() * vel;
        mjo.get_gdC().block<6, 3>(i * 6, 0) += omega * gradViolaPc;
        gdT(i) += omega * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++idx_cp;
        }
      } // end iteration through all constraint points in segment
    } // end iteration through all pieces

    obs_swarm_feas_qvar_costs(3) += var;

    /* Publish visualization of gradients*/
    // visualization_->displayIntermediateGrad("smoothness", ctrl_pts_q_, mjo.get_gdC());
    // visualization_->displayIntermediateGrad("dist_variance", cstr_pts_q_, gdp);

  } // end func addPVAGradCost2CT

  
  bool PolyhedronSFCOptimizer::swarmGradCostP(const double t,
                                         const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp, double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    double pt_time = t_now_ + t;

    // if (swarm_local_trajs_ == nullptr){
    //   return false;
    // }

    swarm_traj_mutex_.lock();  

    for (const auto& id_traj_pair : swarm_local_trajs_){ // Iterate through trajectories

      auto agent_traj = id_traj_pair.second;
      
      if ((agent_traj.drone_id < 0) || agent_traj.drone_id == drone_id_)
      {
        // Ignore if trajectory belongs to self
        continue;
      }

      double traj_i_start_time = agent_traj.start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_start_time + agent_traj.duration)
      {
        swarm_p = agent_traj.traj.getPos(pt_time - traj_i_start_time);
        swarm_v = agent_traj.traj.getVel(pt_time - traj_i_start_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_start_time + agent_traj.duration);
        swarm_v = agent_traj.traj.getVel(agent_traj.duration);
        swarm_p = agent_traj.traj.getPos(agent_traj.duration) +
                  exceed_time * swarm_v;
      }
      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;

        costp += wei_swarm_ * dist2_err3;

        Eigen::Vector3d dJ_dP = wei_swarm_ * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }

      if (min_ellip_dist2_ > ellip_dist2)
      {
        min_ellip_dist2_ = ellip_dist2;
      }
    }

    swarm_traj_mutex_.unlock();  

    return ret;
  }

  void PolyhedronSFCOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    size_t num_pts = ps.cols();
    // dist_btw_all_pts: distance between consecutive points
    //    ps.rightCols(num_pts-1): block containing all columns except first one
    //    ps.rightCols(num_pts-1): block containing all columns except last one
    Eigen::MatrixXd dist_btw_all_pts = ps.rightCols(num_pts - 1 ) - ps.leftCols(num_pts - 1);
    // dsqrs: Get squared norm of dist_btw_all_pts
    Eigen::VectorXd dsqrs = dist_btw_all_pts.colwise().squaredNorm().transpose();
    // dquarsum: Get squared norm of of all dsqrs
    double dquarsum = dsqrs.squaredNorm(); 

    // dquarmean: Mean of all squared distance
    double dquarmean = dquarsum / (num_pts-1);

    var = wei_sqrvar_ * (dquarmean);
    gdp.resize(3, num_pts);
    gdp.setZero();

    for (size_t i = 0; i < num_pts; i++) // iterate through all points 
    {
      if (i != 0) // If not first point
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1)) / (num_pts-1) * dist_btw_all_pts.col(i - 1));
      }
      if (i != (num_pts-1)) // If not last point
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i)) / (num_pts-1) * dist_btw_all_pts.col(i));
      }
    }

    return;
  }


  double PolyhedronSFCOptimizer::distanceCost(void *func_data,
                                              const double* xi,
                                              double* grad_xi, 
                                              const int n)
  {
      void **dataPtrs = (void **)func_data;
      // extract instance data
      const double &dEps = *((const double *)(dataPtrs[0])); // smoothingFactor
      const Eigen::Vector3d &start_pos = *((const Eigen::Vector3d *)(dataPtrs[1])); // start pos
      const Eigen::Vector3d &end_pos = *((const Eigen::Vector3d *)(dataPtrs[2])); // final pos
      const std::vector<Eigen::Matrix3Xd> &vPolys = *((std::vector<Eigen::Matrix3Xd> *)(dataPtrs[3]));              // vertex repr. of polytopes

      double cost = 0.0;
      const int overlaps = vPolys.size() / 2;

      Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
      Eigen::Vector3d a, b, d;
      Eigen::VectorXd r;
      double smoothedDistance;
      for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
      {
        a = i == 0 ? start_pos : b;
        if (i < overlaps)
        {
          k = vPolys[2 * i + 1].cols();
          Eigen::Map<const Eigen::VectorXd> q(xi + j, k);
          r = q.normalized().head(k - 1);
          b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
              vPolys[2 * i + 1].col(0);
        }
        else
        {
          b = end_pos;
        }

        d = b - a;
        smoothedDistance = sqrt(d.squaredNorm() + dEps);
        cost += smoothedDistance;

        if (i < overlaps)
        {
          gradP.col(i) += d / smoothedDistance;
        }
        if (i > 0)
        {
          gradP.col(i - 1) -= d / smoothedDistance;
        }
      }

      Eigen::VectorXd unitQ;
      double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
      for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
      {
          k = vPolys[2 * i + 1].cols();
          Eigen::Map<const Eigen::VectorXd> q(xi + j, k);
          Eigen::Map<Eigen::VectorXd> gradQ(grad_xi + j, k);
          sqrNormQ = q.squaredNorm();
          invNormQ = 1.0 / sqrt(sqrNormQ);
          unitQ = q * invNormQ;
          gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                              unitQ.head(k - 1).array() * 2.0;
          gradQ(k - 1) = 0.0;
          gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

          sqrNormViolation = sqrNormQ - 1.0;
          if (sqrNormViolation > 0.0)
          {
              c = sqrNormViolation * sqrNormViolation;
              dc = 3.0 * c;
              c *= sqrNormViolation;
              cost += c;
              gradQ += dc * 2.0 * q;
          }
      }

      return cost;
  }

} // namespace back_end