

    //printf("Control costs for dim %d = %f\n", d, control_costs[d].sum());

    // add linear costs:
//    control_costs[d] += weight * (linear_control_costs_[d].array() *
//        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_).array()).matrix();

  // this measures the accelerations and squares them
//  for (int d=0; d<num_dimensions_; ++d)
//  {
//    VectorXd params_all = parameters_all_[d];
//    VectorXd costs_all = VectorXd::Zero(num_vars_all_);
//
//    params_all.segment(free_vars_start_index_, num_vars_free_) = parameters[d] + noise[d];
//    VectorXd acc_all = VectorXd::Zero(num_vars_all_);
//    for (int i=0; i<NUM_DIFF_RULES; ++i)
//    {
//      acc_all = differentiation_matrices_[i]*params_all;
//      costs_all += weight * derivative_costs_[i] * (acc_all.array()*acc_all.array()).matrix();
//    }
//
//    control_costs[d] = costs_all.segment(free_vars_start_index_, num_vars_free_);
//    for (int i=0; i<free_vars_start_index_; ++i)
//    {
//      control_costs[d](0) += costs_all(i);
//      control_costs[d](num_vars_free_-1) += costs_all(num_vars_all_-(i+1));
//    }
//  }

//bool CovariantMovementPrimitive::computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices, const std::vector<std::vector<Eigen::VectorXd> >& parameters,
//                                                     const double weight, std::vector<Eigen::VectorXd>& control_costs)
//{
//  //Policy::computeControlCosts(control_cost_matrices, parameters, weight, control_costs);
//
//  // we use the locally stored control costs
//
//  // this uses the already squared control cost matrix
//  /*for (int d=0; d<num_dimensions_; ++d)
//    {
//        control_costs[d] = VectorXd::Zero(num_time_steps_);
//        VectorXd params_all = parameters_all_[d];
//        for (int t=0; t<num_time_steps_; ++t)
//        {
//            params_all.segment(free_vars_start_index_, num_vars_free_) = parameters[d][t];
//            VectorXd r_times_u = control_costs_all_[d] * params_all;
//            control_costs[d] += weight * (r_times_u.segment(free_vars_start_index_, num_vars_free_).cwise() * parameters[d][t]);
//        }
//    }*/
//
//
//  // this measures the accelerations and squares them
//  for (int d=0; d<num_dimensions_; ++d)
//  {
//    VectorXd params_all = parameters_all_[d];
//    VectorXd costs_all = VectorXd::Zero(num_vars_all_);
//    for (int t=0; t<num_time_steps_; ++t)
//    {
//      params_all.segment(free_vars_start_index_, num_vars_free_) = parameters[d][t];
//      VectorXd acc_all = VectorXd::Zero(num_vars_all_);
//      for (int i=0; i<NUM_DIFF_RULES; ++i)
//      {
//        acc_all = differentiation_matrices_[i]*params_all;
//        costs_all += weight * derivative_costs_[i] * (acc_all.array()*acc_all.array()).matrix();
//      }
//    }
//    control_costs[d] = costs_all.segment(free_vars_start_index_, num_vars_free_);
//    for (int i=0; i<free_vars_start_index_; ++i)
//    {
//      control_costs[d](0) += costs_all(i);
//      control_costs[d](num_vars_free_-1) += costs_all(num_vars_all_-(i+1));
//    }
//  }
//
//
//  return true;
//}


    // derivative_costs[d].col(STOMP_VELOCITY) = Eigen::VectorXd::Ones(
    //   num_time_steps_ + 2*TRAJECTORY_PADDING);
    // derivative_costs[d].col(STOMP_ACCELERATION) = 0.01*Eigen::VectorXd::Ones(
    //   num_time_steps_ + 2*TRAJECTORY_PADDING);

    // derivative_costs[d].col(STOMP_POSITION) = 0.0001 * Eigen::VectorXd::Ones(
    //   num_time_steps_ + 2*TRAJECTORY_PADDING);
    // derivative_costs[d](30, STOMP_POSITION) = 1000000.0;
    // initial_trajectory[d](30) = 0.3;
    // derivative_costs[d](80, STOMP_POSITION) = 1000000.0;
    // initial_trajectory[d](80) = 0.8;


//bool CovariantMovementPrimitive::readFromDisc(const std::string abs_file_name)
//{
//    // TODO: implement this
//    return true;
//}


  //    for (int d=0; d<num_dimensions_; ++d)
  //    {
  //      double weight = 0.0;
  //      double weight_sum = 0.0;
  //
  //      Eigen::VectorXd update = Eigen::VectorXd::Zero(num_vars_free_);
  //      for (int t=0; t<num_time_steps_; ++t)
  //      {
  //          weight = time_step_weights[d][t];
  //          weight_sum += weight;
  //          update.transpose() += updates[d].row(t) * weight;
  //          //ROS_INFO_STREAM("Update at time " << t << " = " << updates[d].row(t));
  //      }
  //      if (weight_sum <1e-6)
  //        weight_sum = 1e-6;
  //      parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) += (1.0/weight_sum)*update;
  //    }

  // this weights updates by number of time-steps remaining:
  //    for (int d=0; d<num_dimensions_; ++d)
  //    {
  //        double weight=0.0;
  //        double weight_sum=0.0;
  //        Eigen::VectorXd update = Eigen::VectorXd::Zero(num_vars_free_);
  //        for (int t=0; t<num_time_steps_; ++t)
  //        {
  //            weight = double(num_time_steps_ - t);
  //            weight_sum += weight;
  //            update.transpose() += updates[d].row(t) * weight;
  //            //ROS_INFO_STREAM("Update at time " << t << " = " << updates[d].row(t));
  //        }
  //        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) += (1.0/weight_sum)*update;
  //    }


  // this takes only the diagonal elements
  /*for (int d=0; d<num_dimensions_; ++d)
    {
        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) += updates[d].diagonal();
    }*/


##################################################################
from stomp_2d_test.cpp/h
double evaluateCostPath(
  double x1, double y1, double x2, double y2, double vx, double vy) const;

double StompTest::evaluateCostPath(
  double x1, double y1,
  double x2, double y2,
  double vx, double vy) const {
  double ax = 0.0, ay = 0.0, gx = 0.0, gy = 0.0;
  return interpPathSegmentAndEvaluateStateCost(
    x1, y1, x2, y2, vx, vy, false, ax, ay, gx, gy);
}

##################################################################
from stomp_2d_test.cpp/h

    // 2020-01-01 semantics:
    // sample the discretized 2D space
    // with resolution (dx/y) / num_samples
    double d = (static_cast<double>(i) /
      static_cast<double>(num_samples));
    double x = x1 + d * delta(0);
    double y = y1 + d * dy;

##################################################################

void readParameters();

// void StompTest::readParameters() {
//   YAML::Node config = YAML::LoadFile(
//     "/home/jim/Dev/jim/stomp/stomp/test/stomp_2d_test.yaml");

//   // WARNING, TODO: no error checking here!!!
//   obstacles_.clear();
//   XmlRpc::XmlRpcValue obstacles_xml;
//   STOMP_VERIFY(node_handle_.getParam("cost_function", obstacles_xml));
//   for (int i = 0; i < obstacles_xml.size(); ++i) {
//     Obstacle o;
//     STOMP_VERIFY(getParam(obstacles_xml[i], "center", o.center_));
//     STOMP_VERIFY(getParam(obstacles_xml[i], "radius", o.radius_));
//     STOMP_VERIFY(getParam(obstacles_xml[i], "boolean", o.inadmissible_));
//     obstacles_.push_back(o);
//   }

//   // STOMP_VERIFY(node_handle_.getParam("num_iterations", num_iterations_));
//   // if (config["num_iterations"]) {
//   //   num_iterations_ = config["num_iterations"].as<int>();
//   //   std::cout << "num_iterations found!!! " << num_iterations_ << "\n";
//   // }

//   STOMP_VERIFY(node_handle_.getParam("num_time_steps", num_time_steps_));
//   STOMP_VERIFY(node_handle_.getParam("movement_duration", movement_duration_));
//   STOMP_VERIFY(node_handle_.getParam(
//     "control_cost_weight", control_cost_weight_));
//   STOMP_VERIFY(node_handle_.getParam("output_dir", output_dir_));
//   STOMP_VERIFY(node_handle_.getParam("use_chomp", use_chomp_));
//   STOMP_VERIFY(node_handle_.getParam(
//     "save_noisy_trajectories", save_noisy_trajectories_));
//   STOMP_VERIFY(node_handle_.getParam(
//     "save_noiseless_trajectories", save_noiseless_trajectories_));
//   STOMP_VERIFY(node_handle_.getParam(
//     "save_cost_function", save_cost_function_));
//   STOMP_VERIFY(node_handle_.getParam(
//     "publish_to_rviz", publish_to_rviz_));
//   STOMP_VERIFY(node_handle_.getParam(
//     "delay_per_iteration", delay_per_iteration_));

//   STOMP_VERIFY(readDoubleArray(node_handle_,
//     "params_s", params_s_));
//   STOMP_VERIFY(readDoubleArray(node_handle_,
//     "params_e", params_e_));
//   STOMP_VERIFY(node_handle_.getParam(
//     "num_dimensions", num_dimensions_));
//   // 2019-12-31 TODO(jim) consolidate starting(ending)_point / num_dimensions
//   // to infer num_dimensions_ from size of params_s etc.
// }

##################################################################

    if (use_covariance_matrix_adaptation_) {
      // printf("use_covariance_matrix_adaptation_ is True\n");
      // true CMA method
      // adapted_covariances_[d] = Eigen::MatrixXd::Zero(num_time_steps_, num_time_steps_);
      // for (int r=0; r<num_rollouts_; ++r)
      // {
      //   adapted_covariances_[d] += rollouts_[r].full_probabilities_[d] *
      //       rollouts_[r].noise_[d] * rollouts_[r].noise_[d].transpose();
      // }

      // ROS_INFO_STREAM("Covariance for dimension " << d << " = " << adapted_covariances_[d]);
      // adapted_stddevs_[d] = 1.0;
      // adapted_covariance_inverse_[d] = adapted_covariances_[d].fullPivLu().inverse();
      // noise_generators_[d] = MultivariateGaussian(VectorXd::Zero(num_parameters_[d]), adapted_covariances_[d]);

      // one-dimensional CMA-ish
      // double var = 0.0;
      // for (int r=0; r<num_rollouts_; ++r)
      // {
      //   double dist = rollouts_[r].noise_[d].transpose() *
      //       adapted_covariance_inverse_[d] * rollouts_[r].noise_[d];
      //   var += rollouts_[r].full_probabilities_[d] * dist;
      //   //printf("Rollout %d, dist = %f", r, dist);
      // }
      // var /= num_time_steps_;
      // adapted_stddevs_[d] = 0.8 * adapted_stddevs_[d] + 0.2 * sqrt(var);
      // ROS_INFO("Dimension %d: new stddev = %f", d, adapted_stddevs_[d]);

      double frob_stddev = 0.0, numer = 0.0, denom = 0.0;

      /*
      // true CMA method + minimization of frobenius norm
      adapted_covariances_[d] = Eigen::MatrixXd::Zero(num_time_steps_, num_time_steps_);
      for (int r=0; r<num_rollouts_; ++r)
      {
        adapted_covariances_[d] += rollouts_[r].full_probabilities_[d] *
            rollouts_[r].noise_[d] * rollouts_[r].noise_[d].transpose();
      }

      // minimize frobenius norm of diff between a_c and std_dev^2 * inv_control_cost
      numer = 0.0;
      denom = 0.0;
      for (int i=0; i<num_time_steps_; ++i)
      {
        for (int j=0; j<num_time_steps_; ++j)
        {
          numer += adapted_covariances_[d](i,j) * inv_control_costs_[d](i,j);
          denom += inv_control_costs_[d](i,j) * inv_control_costs_[d](i,j);
        }
      }
      frob_stddev = sqrt(numer/denom);
      double prev_frob_stddev = frob_stddev;
      */

##################################################################

double StompTest::evaluateCost(
  Eigen::MatrixXd* param_sample,
  double vx, double vy) const {
  double ax = 0.0, ay = 0.0, gx = 0.0, gy = 0.0;
  return evaluateStateCostWithGradients(
    param_sample,
    vx, vy,
    false,
    ax, ay,
    gx, gy);
}


  double evaluateCost(
    Eigen::MatrixXd* param_sample,
    double vx, double vy) const;
  // calls evaluateStateCostWithGradients


##################################################################


# list of cost functions for large scale testing:
cost_functions:
  # simple cost function, offset
  offset:
    -
      center: [0.6, 0.4]
      radius: [0.5, 0.5]
      boolean: false

  # simple cost function, centered
  centered:
    -
      center: [0.5, 0.5]
      radius: [0.3, 0.3]
      boolean: false

  maze:
    -
      center: [0.3, 0.0]
      radius: [0.1, 0.7]
      boolean: false
    -
      center: [0.7, 1.0]
      radius: [0.1, 0.7]
      boolean: false

  saddle:
    -
      center: [0.60, 0.40]
      radius: [0.20, 0.20]
      boolean: false
    -
      center: [0.40, 0.60]
      radius: [0.20, 0.20]
      boolean: false

##################################################################

##################################################################

##################################################################

##################################################################

##################################################################

##################################################################

##################################################################