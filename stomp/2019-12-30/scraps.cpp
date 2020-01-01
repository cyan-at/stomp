

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




