

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

void writeCostFunction();

void StompTest::writeCostFunction() {
  std::stringstream ss;
  ss << output_dir_ << "/cost_function.txt";
  int num_x = lrint(1.0 / resolution_) + 1;
  int num_y = lrint(1.0 / resolution_) + 1;

  FILE *f = fopen(ss.str().c_str(), "w");
  fprintf(f, "%d\t%d\n", num_x, num_y);

  Eigen::MatrixXd param_sample = Eigen::MatrixXd::Zero(num_dimensions_, 1);
  for (int i = 0; i < num_x; ++i) {
    double x = i*resolution_;
    for (int j = 0; j < num_y; ++j) {
      double y = j*resolution_;

      param_sample(0, 0) = x;
      param_sample(1, 0) = y;
      double cost = (this->*evaluateStateCostStrategy)(&param_sample);
      fprintf(f, "%lf\t%lf\t%lf\n", x, y, cost);
    }
  }
  fclose(f);
}

##################################################################

int StompTest::run(
  std::vector<std::vector<double>>* obstacle_points) {
  obstacle_points_ = obstacle_points;

  srand(time(NULL));
  mkdir(output_dir_.c_str(), 0755);
  std::stringstream stddev_filename, cost_filename;
  std::stringstream num_rollouts_filename;
  stddev_filename << output_dir_ << "/stddevs.txt";
  cost_filename << output_dir_ << "/costs.txt";
  num_rollouts_filename << output_dir_ << "/num_rollouts.txt";
  FILE *stddev_file = fopen(stddev_filename.str().c_str(), "w");
  FILE *cost_file = fopen(cost_filename.str().c_str(), "w");
  FILE *num_rollouts_file = NULL;
  if (save_noisy_trajectories_) {
    num_rollouts_file = fopen(num_rollouts_filename.str().c_str(), "w");
  }

  std::vector<Eigen::VectorXd> initial_trajectory;
  initial_trajectory.resize(num_dimensions_,
    Eigen::VectorXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING));

  std::vector<Eigen::MatrixXd> derivative_costs;
  derivative_costs.resize(
    num_dimensions_,
    Eigen::MatrixXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING,
      NUM_DIFF_RULES));

  for (int d = 0; d < num_dimensions_; ++d) {
    // apply starting point and ending point here
    initial_trajectory[d].head(TRAJECTORY_PADDING) =
      params_s_[d]*Eigen::VectorXd::Ones(
      TRAJECTORY_PADDING);
    initial_trajectory[d].tail(TRAJECTORY_PADDING) =
      params_e_[d]*Eigen::VectorXd::Ones(
      TRAJECTORY_PADDING);

    derivative_costs[d].col(STOMP_ACCELERATION) = Eigen::VectorXd::Ones(
      num_time_steps_ + 2*TRAJECTORY_PADDING);
  }

  policy_.reset(new CovariantMovementPrimitive());
  policy_->initialize(
    num_time_steps_,
    num_dimensions_,
    movement_duration_,
    derivative_costs,
    initial_trajectory);
  policy_->setToMinControlCost();
  policy_->getParametersAll(initial_trajectory_);

  vel_diff_matrix_ = policy_->getDifferentiationMatrix(
    stomp::STOMP_VELOCITY);
  acc_diff_matrix_ = policy_->getDifferentiationMatrix(
    stomp::STOMP_ACCELERATION);
  movement_dt_ = policy_->getMovementDt();

  if (save_cost_function_)
    writeCostFunction();
  if (publish_to_rviz_) {
    int timeout = 5;
    int counter = 0;
    while (rviz_pub_.getNumSubscribers() == 0 && counter < timeout) {
      ROS_WARN("Waiting for rviz to connect...");
      ros::Duration(1.0).sleep();
      ++counter;
    }
    (this->*visualizeCostFunctionStrategy)();
  }

  ros::NodeHandle stomp_node_handle(node_handle_, "stomp");
  // stomp_.reset(new stomp::STOMP());
  boost::shared_ptr<stomp::STOMP> stomp_boost_sharedptr(&stomp,
      &stomp::null_deleter<stomp::STOMP>);
  stomp_ = stomp_boost_sharedptr;
  stomp_->initialize(num_dimensions_,
    stomp_node_handle, shared_from_this());

  // ros::NodeHandle chomp_node_handle(node_handle_, "chomp");
  // chomp_.reset(new stomp::CHOMP());
  // chomp_->initialize(chomp_node_handle, shared_from_this());

  if (save_noiseless_trajectories_) {
    std::stringstream sss;
    sss << output_dir_ << "/noiseless_0.txt";
    policy_->writeToFile(sss.str());
  }

  CovariantMovementPrimitive tmp_policy = *policy_;

  ros::Time prev_iter_stamp = ros::Time::now();

  const clock_t begin_time = std::clock();

  double latest_trajectory_cost = 0.0;
  for (int i = 1; i <= num_iterations_; ++i) {
    std::vector<Rollout> rollouts;
    Rollout noiseless_rollout;
    if (use_chomp_) {
      // chomp_->runSingleIteration(i);
      // chomp_->getNoiselessRollout(noiseless_rollout);
      // if (save_noiseless_trajectories_) {
      //   for (unsigned int d = 0; d < num_dimensions_; ++d) {
      //     fprintf(stddev_file, "%f\t", 0.0);
      //   }
      //   fprintf(stddev_file, "\n");
      // }
    } else {
      // printf("running single iteration\n");
      stomp_->runSingleIteration(i);
      stomp_->getAllRollouts(rollouts);
      stomp_->getNoiselessRollout(noiseless_rollout);

      std::vector<double> stddevs;
      stomp_->getAdaptedStddevs(stddevs);
      if (save_noiseless_trajectories_) {
        for (unsigned int d = 0; d < stddevs.size(); ++d) {
          fprintf(stddev_file, "%f\t", stddevs[d]);
        }
        fprintf(stddev_file, "\n");
      }
    }

    if (save_noiseless_trajectories_) {
      std::stringstream ss;
      ss << output_dir_ << "/noiseless_" << i << ".txt";
      policy_->writeToFile(ss.str());
      fprintf(cost_file, "%f\n", noiseless_rollout.total_cost_);
    }

    if (save_noisy_trajectories_) {
      fprintf(num_rollouts_file, "%d\n", static_cast<int>(rollouts.size()));
      for (unsigned int j = 0; j < rollouts.size(); ++j) {
        std::stringstream ss2;
        ss2 << output_dir_ << "/noisy_" << i << "_" << j << ".txt";
        // tmp_policy.setParameters(rollouts[j].parameters_noise_projected_);
        tmp_policy.setParameters(rollouts[j].parameters_noise_);
        tmp_policy.writeToFile(ss2.str());
      }
    }
    // printf("%f\n", noiseless_rollout.total_cost_);

    // if (publish_to_rviz_) {
    if (true) {
      // wait until delay_per_iteration
      double delay = 0.0;
      // while (delay < delay_per_iteration_) {
      //   delay = (ros::Time::now() - prev_iter_stamp).toSec();
      //   if (delay < delay_per_iteration_) {
      //     ros::Duration(delay_per_iteration_ - delay).sleep();
      //   }
      // }
      // prev_iter_stamp = ros::Time::now();

      (this->*visualizeTrajectoryStrategy)(noiseless_rollout, true, 0);
      if (!use_chomp_) {
        for (size_t j=0; j < rollouts.size(); ++j) {
          (this->*visualizeTrajectoryStrategy)(rollouts[j], false, j+1);
        }
      }
    }
  }

  double runtime_secs = static_cast<double>(
    std::clock() - begin_time) / CLOCKS_PER_SEC;
  printf("total runtime_secs %.3f\n", runtime_secs);

  fclose(stddev_file);
  fclose(cost_file);
  if (save_noisy_trajectories_)
    fclose(num_rollouts_file);

  std::string filename = "/home/jim/Desktop/stomp_output.yaml";
  saveTrajectoryStrategy3(filename.c_str());

  stomp_.reset();
  // chomp_.reset();
  policy_.reset();

  return 0;
}

##################################################################

##################################################################

##################################################################

##################################################################

##################################################################