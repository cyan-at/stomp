/*
 * stomp_2d_test.cpp
 *  Copyright (c) 2010, Willow Garage, Inc.
 *
 *  Created on: Feb 2, 2012
 *      Author: kalakris
 */

#include <stomp/stomp_test.h>

namespace stomp {

bool StompTest::HandlePlanStomp(
  HandlePlanStompSrv::Request& req,    // NOLINT(runtime/references)
  HandlePlanStompSrv::Response& res) {  // NOLINT(runtime/references)
  printf("StompTest::HandlePlanStomp\n");
  // std::cout << req.params_s << std::endl;

  printf("all params_s: ");
  for (int i = 0; i << req.params_s.size(); ++i) {
    printf("%.3f", req.params_s[i]);
  }
  printf("\n");

  return true;
}

bool StompTest::initialize(int num_threads, int num_rollouts) {
  std::vector<Eigen::VectorXd> initial_trajectory;
  initial_trajectory.resize(num_dimensions_,
    Eigen::VectorXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING));

  std::vector<Eigen::MatrixXd> derivative_costs;
  derivative_costs.resize(
    num_dimensions_,
    Eigen::MatrixXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING,
      NUM_DIFF_RULES));

  // go from joint space to tool space
  analytic_ur_fk(&joints,
    &params_s_,
    // TODO(jim) rewrite parameters_noise_
    // to eigen matrix
    &fk_hom,
    &gripper_fixed_hom);
  params_s_[0] = fk_hom(0, 3);
  params_s_[1] = fk_hom(1, 3);
  params_s_[2] = fk_hom(2, 3);
  analytic_ur_fk(&joints,
    &params_e_,
    // TODO(jim) rewrite parameters_noise_
    // to eigen matrix
    &fk_hom,
    &gripper_fixed_hom);
  params_e_[0] = fk_hom(0, 3);
  params_e_[1] = fk_hom(1, 3);
  params_e_[2] = fk_hom(2, 3);

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

  return true;
}

int StompTest::run(
  std::vector<std::vector<double>>* obstacle_points) {
  obstacle_points_ = obstacle_points;

  initialized_ = initialize(0, 0);

  int counter = 0;
  while (rviz_pub_.getNumSubscribers() == 0 && counter < 5) {
    ROS_WARN("Waiting for rviz to connect...");
    ros::Duration(1.0).sleep();
    ++counter;
  }
  (this->*visualizeCostFunctionStrategy)();

  ros::NodeHandle stomp_node_handle(node_handle_, "stomp");
  boost::shared_ptr<stomp::STOMP> stomp_boost_sharedptr(&stomp,
      &stomp::null_deleter<stomp::STOMP>);
  stomp_ = stomp_boost_sharedptr;
  stomp_->initialize(num_dimensions_,
    stomp_node_handle, shared_from_this());

  const clock_t begin_time = std::clock();

  for (int i = 1; i <= num_iterations_; ++i) {
    std::vector<Rollout> rollouts;
    stomp_->runSingleIteration(i);
    stomp_->getAllRollouts(rollouts);
    for (size_t j=0; j < rollouts.size(); ++j) {
      (this->*visualizeTrajectoryStrategy)(rollouts[j], false, j+1);
    }

    Rollout noiseless_rollout;
    stomp_->getNoiselessRollout(noiseless_rollout);
    // printf("%f\n", noiseless_rollout.total_cost_);
    (this->*visualizeTrajectoryStrategy)(noiseless_rollout, true, 0);

    ros::Duration(0.1).sleep();
  }

  double runtime_secs = static_cast<double>(
    std::clock() - begin_time) / CLOCKS_PER_SEC;
  printf("total runtime_secs %.3f\n", runtime_secs);

  std::string filename = "/home/jim/Desktop/stomp_output.yaml";
  saveTrajectoryStrategy3(filename.c_str());

  stomp_.reset();
  policy_.reset();

  return 0;
}

bool StompTest::execute(const std::vector<Eigen::VectorXd>& parameters,
  const std::vector<Eigen::VectorXd>& projected_parameters,
  Eigen::VectorXd* costs,
  Eigen::MatrixXd* weighted_feature_values,
  const int iteration_number,
  const int rollout_number,
  int thread_id,
  bool compute_gradients,
  std::vector<Eigen::VectorXd>& gradients,
  bool& validity) {
  *costs = Eigen::VectorXd::Zero(num_time_steps_);

  // ##################################################################
  // weighted_feature_values = Eigen::MatrixXd::Zero(num_time_steps_, 1);

  Eigen::MatrixXd pos(num_dimensions_,
    num_time_steps_ + 2*TRAJECTORY_PADDING);
  Eigen::MatrixXd vel(num_dimensions_,
    num_time_steps_ + 2*TRAJECTORY_PADDING);
  Eigen::MatrixXd acc(num_dimensions_,
    num_time_steps_ + 2*TRAJECTORY_PADDING);

  Eigen::MatrixXd proj_pos(num_dimensions_,
    num_time_steps_ + 2*TRAJECTORY_PADDING);

  for (int d = 0; d < num_dimensions_; ++d) {
    // printf("lvalue size = %d, %d, rvalue size = %d, %d", 1,
    //   num_time_steps_, projected_parameters[d].rows(),
    //   projected_parameters[d].cols());
    proj_pos.row(d) = initial_trajectory_[d];
    proj_pos.block(d, TRAJECTORY_PADDING,
      1, num_time_steps_) = projected_parameters[d].transpose();

    pos.row(d) = initial_trajectory_[d];
    pos.block(d, TRAJECTORY_PADDING,
      1, num_time_steps_) = parameters[d].transpose();
  }

  if (compute_gradients) {
    gradients.resize(num_dimensions_,
      Eigen::VectorXd::Zero(num_time_steps_));
  }
  vel = (vel_diff_matrix_ * pos.transpose()).transpose();
  if (compute_gradients) {
    acc = (acc_diff_matrix_ * pos.transpose()).transpose();
  }
  // ##################################################################

  // 2019-12-26 COMPUTE COSTS HERE!!!
  // Expression 2(a) in paper, S(theta_k_i), S(t_i)
  // cost of path, q(theta_k_i) state cost

  Eigen::MatrixXd last_param_sample(num_dimensions_, 1);
  last_param_sample = pos.col(0);

  Eigen::MatrixXd this_param_sample(num_dimensions_, 1);

  for (int t = TRAJECTORY_PADDING;
    t < TRAJECTORY_PADDING+num_time_steps_; ++t) {
    // compute the cost for this rollout
    this_param_sample = pos.col(t);

    double cost = 0.0;
    if (compute_gradients) {
      cost = interpPathSegmentAndEvaluateStateCost(
        &last_param_sample,
        &this_param_sample,
        vel(0, t), vel(1, t),
        true,
        acc(0, t), acc(1, t),
        gradients[0](t-TRAJECTORY_PADDING),
        gradients[1](t-TRAJECTORY_PADDING));
    } else {
      double gx = 0.0, gy = 0.0;
      // 2020-01-01 gx/y need to be non-temporary
      // because they are refs
      cost = interpPathSegmentAndEvaluateStateCost(
        &last_param_sample,
        &this_param_sample,
        vel(0, t), vel(1, t),
        false,
        0.0, 0.0,
        gx, gy);
    }
    (*costs)(t-TRAJECTORY_PADDING) = cost;

    last_param_sample = pos.col(t);
  }
  validity = true;
  return true;
}

double StompTest::interpPathSegmentAndEvaluateStateCost(
  Eigen::MatrixXd* last_param_sample,
  Eigen::MatrixXd* this_param_sample,
  double vx, double vy,
  bool compute_gradients,
  double ax, double ay,
  double& gx, double& gy) {

  // compute norm, get num_samples by resolution_
  Eigen::MatrixXd delta = this_param_sample->col(0) - last_param_sample->col(0);
  // PER DIMENSION, keep signage for interp

  double total_delta_norm = delta.norm();
  // total across all dimensions
  // in 3 dimensions, you can see this as
  // frob. norm or 3D line dist

  int num_samples = ceil(total_delta_norm / resolution_);
  num_samples = num_samples > 20 ? 20 : num_samples;
  // TODO(jim) expose this arb. param 20 to yaml?

  if (compute_gradients) {
    gx = 0.0;
    gy = 0.0;
  }

  double cost = 0.0;
  Eigen::MatrixXd interp_sample = Eigen::MatrixXd::Zero(num_dimensions_, 1);
  for (int i = 0; i < num_samples; ++i) {
    // leave out the last one to avoid double counting
    // explanation:
    // final i will be (num_samples - 1) / num_samples
    // so does not evaluate cost @ exactly x2/y2
    // aka this_param_sample
    // that will be done in the next call to interpPathSeg
    // on i = 0

    // note that this technique
    // means we DO interp across the dim numerical space
    // evenly, but for some dims where delta[dim] / resolution > num_samples
    // we are UNDERsampling (actual resolution > resolution_)
    // and other dims where delta[dim] / resolution  < num_samples
    // we are OVERsampling (actual resolution < resolution_)
    // but we do do so evenly

    double interp_ratio = (static_cast<double>(i) /
      static_cast<double>(num_samples));

    interp_sample = (*last_param_sample) + interp_ratio * delta;    
    // for (int d = 0; d < num_dimensions_; ++d) {
    //   interp_sample(d, 0) = (*last_param_sample)(d, 0) +\
    //     interp_ratio * delta(d, 0);
    // }

    double temp_gx = 0.0, temp_gy = 0.0;
    cost += evaluateStateCostWithGradients(
      &interp_sample,
      vx, vy,
      compute_gradients,
      ax, ay,
      temp_gx, temp_gy);
    gx += temp_gx;
    gy += temp_gy;
  }

  cost /= num_samples;
  // normalize it by num_samples
  // so semantically same for different sized paths

  if (compute_gradients) {
    gx /= num_samples;
    gy /= num_samples;
  }

  return cost;
}

double StompTest::evaluateStateCostWithGradients(
  Eigen::MatrixXd* param_sample,
  double vx, double vy,
  bool compute_gradients,
  double ax, double ay,
  double& gx, double& gy) {
  double cost = (this->*evaluateStateCostStrategy)(param_sample) * movement_dt_;

  double vel_mag = sqrt(vx*vx + vy*vy);

  /*
  if (compute_gradients) {
    double map_gx = 0.0, map_gy = 0.0;
    evaluateStateCostGradientStrategy1(x, y, map_gx, map_gy);

    map_gx *= movement_dt_;
    map_gy *= movement_dt_;

    Eigen::Vector2d vel;
    Eigen::Vector2d norm_vel;
    vel(0) = vx;
    vel(1) = vy;
    norm_vel = vel.normalized();
    Eigen::Matrix2d orth_proj =
      Eigen::Matrix2d::Identity() - norm_vel*norm_vel.transpose();
    Eigen::Vector2d acc;
    acc(0) = ax;
    acc(1) = ay;
    Eigen::Vector2d curvature = (1.0/vel.squaredNorm()) * orth_proj * acc;
    Eigen::Vector2d grad;
    grad(0) = map_gx;
    grad(1) = map_gy;
    Eigen::Vector2d new_grad = vel_mag * (orth_proj*grad - cost*curvature);
    gx = new_grad(0);
    gy = new_grad(1);
  }
  */

  return cost * vel_mag;
}

bool StompTest::filter(std::vector<Eigen::VectorXd>& parameters,
  int thread_id) const {
  return false;
  bool filtered = false;
  for (unsigned int d = 0; d < parameters.size(); ++d) {
    for (int t = 0; t < num_time_steps_; ++t) {
      if (parameters[d](t) < 0.0) {
        parameters[d](t) = 0.0;
        filtered = true;
      }
      if (parameters[d](t) > 1.0) {
        parameters[d](t) = 1.0;
        filtered = true;
      }
    }
  }
  return filtered;
}

bool StompTest::getPolicy(
  boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy) {
  policy = policy_;
  return true;
}

bool StompTest::setPolicy(
  const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy) {
  policy_ = policy;
  return true;
}

double StompTest::getControlCostWeight() {
  return control_cost_weight_;
}

////////////////////////////////// STRATEGY SETS

//////////////////////////////////

double StompTest::evaluateStateCostStrategy1(
  Eigen::MatrixXd* param_sample) {
  double cost = 0.0;

  double x = (*param_sample)(0, 0);
  double y = (*param_sample)(1, 0);

  for (unsigned int o = 0; o < collision_geometries_.size(); ++o) {
    double dx = (x - collision_geometries_[o].center_[0])
      / collision_geometries_[o].radius_[0];
    double dy = (y - collision_geometries_[o].center_[1])
      / collision_geometries_[o].radius_[1];

    double dist = dx * dx + dy * dy;

    // 2020-01-01 semantics:
    // if within the radius from the center
    // of a 'True' obstacle
    // cost is ticked to 1
    // 'inadmissible'

    // if within the radius from the center
    // of a 'False' obstacle
    // if cost is 0.0
    // it is raised to a partial
    // 'admissible' obstacle
    if (collision_geometries_[o].inadmissible_) {
      if (dist < 1.0) {
        // cost += 1.0;
        if (cost < 1.0)
          cost = 1.0;
      }
    } else {
      if (dist < 1.0) {
        // cost += 1.0 - dist;
        if (cost < 1.0 - dist)
          cost = 1.0 - dist;
      }
    }
  }
  // TODO(jim) generalize this to 3D obstacles

  // joint limits
  // TODO(jim) generalize this to n dimensions
  // parsed from YAML file
  // as well as cost for exceeding
  // imposing a 'joint' limit voxel of 1x1 fro 0-1
  const double joint_limit_cost = 100.0;
  if (x < 0.0) {
    cost += joint_limit_cost * -x;
  }
  if (x > 1.0) {
    cost += joint_limit_cost * (x - 1.0);
  }
  if (y < 0.0) {
    cost += joint_limit_cost * -y;
  }
  if (y > 1.0) {
    cost += joint_limit_cost * (y - 1.0);
  }

  return cost;
}

void StompTest::evaluateStateCostGradientStrategy1(
  Eigen::MatrixXd* param_sample,
  double& gx, double& gy) {
  // average map cost / map area (2 * resolution big)
  // gx = (evaluateStateCostStrategy1(x + resolution_, y)
  //   - evaluateStateCostStrategy1(x - resolution_, y)) / (2 * resolution_);
  // gy = (evaluateStateCostStrategy1(x, y + resolution_)
  //   - evaluateStateCostStrategy1(x, y - resolution_)) / (2 * resolution_);
}

void StompTest::visualizeCostFunctionStrategy1() {
  visualization_msgs::Marker marker;

  int num_x = lrint(1.0 / resolution_) + 1;
  int num_y = lrint(1.0 / resolution_) + 1;

  marker.id = 0;
  marker.ns = "cost";
  marker.header.frame_id = "BASE";
  marker.header.stamp = ros::Time::now();
  marker.type = marker.CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = resolution_*2.0;
  marker.scale.y = resolution_*2.0;
  marker.scale.z = resolution_*2.0;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.points.reserve(num_x*num_y);
  marker.colors.reserve(num_x*num_y);

  std_msgs::ColorRGBA color;
  geometry_msgs::Point point;

  double min_cost = std::numeric_limits<double>::max();
  double max_cost = std::numeric_limits<double>::min();

  Eigen::MatrixXd param_sample = Eigen::MatrixXd::Zero(num_dimensions_, 1);
  for (int i = 0; i < num_x; ++i) {
    double x = i*resolution_;
    for (int j = 0; j < num_y; ++j) {
      double y = j*resolution_;

      param_sample(0, 0) = x;
      param_sample(1, 0) = y;
      double cost = (this->*evaluateStateCostStrategy)(&param_sample);
      if (cost > max_cost)
        max_cost = cost;
      if (cost < min_cost)
        min_cost = cost;
      point.x = x;
      point.y = y;
      point.z = cost;  // temp storage
      marker.points.push_back(point);
    }
  }

  // now loop and set colors based on the scaling
  for (size_t i = 0; i < marker.points.size(); ++i) {
    double cost = marker.points[i].z;
    double scaled_cost = (cost - min_cost)/(max_cost - min_cost);
    color.r = scaled_cost;
    color.b = 0.5 * (1.0 - scaled_cost);
    color.g = 0.0;
    color.a = 1.0;
    marker.colors.push_back(color);
    marker.points[i].z = 0.1 * scaled_cost;

    // interchange axes x and z
    // double x = marker.points[i].z;
    // marker.points[i].z = marker.points[i].x;
    // marker.points[i].x = x;
  }

  rviz_pub_.publish(marker);
}

void StompTest::visualizeTrajectoryStrategy1(
  Rollout& rollout, bool noiseless, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "BASE";
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajectory";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(num_time_steps_);

  // marker.colors.resize(num_time_steps_);

  Eigen::MatrixXd param_sample = Eigen::MatrixXd::Zero(num_dimensions_, 1);
  for (int t = 0; t < num_time_steps_; ++t) {
    marker.points[t].x = rollout.parameters_noise_[0][t];
    marker.points[t].y = rollout.parameters_noise_[1][t];

    param_sample(0, 0) = rollout.parameters_noise_[0][t];
    param_sample(1, 0) = rollout.parameters_noise_[1][t];
    double cost = (this->*evaluateStateCostStrategy)(&param_sample);

    // 2020-01-02 show noisy path timestep
    // cost as z value for now
    // note that this applies for the optimal path as well
    // it is rendered below 'noiseless', qualitatively looks
    // in-plane costs are so low
    marker.points[t].z = cost;
    marker.points[t].z += 0.005;
    // spacing offset to render out-of-plane
  }

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  if (noiseless) {
    marker.scale.x = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  } else {
    marker.scale.x = 0.002;
    marker.color.a = 0.7;
    marker.color.r = 0.2;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  }

  rviz_pub_.publish(marker);
}

//////////////////////////////////

double StompTest::evaluateStateCostStrategy2(
  Eigen::MatrixXd* param_sample) {
  double cost = 0.0;

  double x = (*param_sample)(0, 0);
  double y = (*param_sample)(1, 0);
  double z = (*param_sample)(2, 0);

  for (unsigned int o = 0; o < collision_geometries_.size(); ++o) {
    double dx = (x - collision_geometries_[o].center_[0])
      / collision_geometries_[o].radius_[0];
    double dy = (y - collision_geometries_[o].center_[1])
      / collision_geometries_[o].radius_[1];
    double dz = (z - collision_geometries_[o].center_[2])
      / collision_geometries_[o].radius_[2];
    double dist = sqrt(dx * dx + dy * dy + dz * dz);

    // 2020-01-01 semantics:
    // if within the radius from the center
    // of a 'True' obstacle
    // cost is ticked to 1
    // 'inadmissible'

    // if within the radius from the center
    // of a 'False' obstacle
    // if cost is 0.0
    // it is raised to a partial
    // 'admissible' obstacle
    if (collision_geometries_[o].inadmissible_) {
      if (dist < 1.0) {
        // cost += 1.0;
        if (cost < 1.0)
          cost = 1.0;
      }
    } else {
      if (dist < 1.0) {
        // cost += 1.0 - dist;
        if (cost < 1.0 - dist)
          cost = 1.0 - dist;
      }
    }
  }

  // joint limits
  // TODO(jim) generalize this to n dimensions
  // parsed from YAML file
  // as well as cost for exceeding
  // imposing a 'joint' limit voxel of 1x1x1 fro 0-1
  const double joint_limit_cost = 100.0;
  if (x < 0.0) {
    cost += joint_limit_cost * -x;
  }
  if (x > 1.0) {
    cost += joint_limit_cost * (x - 1.0);
  }
  if (y < 0.0) {
    cost += joint_limit_cost * -y;
  }
  if (y > 1.0) {
    cost += joint_limit_cost * (y - 1.0);
  }
  if (z < 0.0) {
    cost += joint_limit_cost * -z;
  }
  if (z > 1.0) {
    cost += joint_limit_cost * (z - 1.0);
  }

  return cost;
}

void StompTest::visualizeCostFunctionStrategy2() {
  for (int obstacle_i = 0; obstacle_i < collision_geometries_.size(); ++obstacle_i) {
    visualization_msgs::Marker marker;

    marker.id = obstacle_i;
    marker.ns = "cost";
    marker.header.frame_id = "BASE";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = collision_geometries_[obstacle_i].center_[0];
    marker.pose.position.y = collision_geometries_[obstacle_i].center_[1];
    marker.pose.position.z = collision_geometries_[obstacle_i].center_[2];

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = collision_geometries_[obstacle_i].radius_[0];
    marker.scale.y = collision_geometries_[obstacle_i].radius_[1];
    marker.scale.z = collision_geometries_[obstacle_i].radius_[2];

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;


    // marker.scale.x = resolution_*2.0;
    // marker.scale.y = resolution_*2.0;
    // marker.scale.z = resolution_*2.0;

    // marker.pose.position.x = 0.0;
    // marker.pose.position.y = 0.0;
    // marker.pose.position.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.points.reserve(num_x*num_y);
    // marker.colors.reserve(num_x*num_y);

    // std_msgs::ColorRGBA color;
    // geometry_msgs::Point point;

    // double min_cost = std::numeric_limits<double>::max();
    // double max_cost = std::numeric_limits<double>::min();

    // Eigen::MatrixXd param_sample = Eigen::MatrixXd::Zero(num_dimensions_, 1);
    // for (int i = 0; i < num_x; ++i) {
    //   double x = i*resolution_;
    //   for (int j = 0; j < num_y; ++j) {
    //     double y = j*resolution_;

    //     param_sample(0, 0) = x;
    //     param_sample(1, 0) = y;
    //     double cost = (this->*evaluateStateCostStrategy)(&param_sample);
    //     if (cost > max_cost)
    //       max_cost = cost;
    //     if (cost < min_cost)
    //       min_cost = cost;
    //     point.x = x;
    //     point.y = y;
    //     point.z = cost;  // temp storage
    //     marker.points.push_back(point);
    //   }
    // }

    // // now loop and set colors based on the scaling
    // for (size_t i = 0; i < marker.points.size(); ++i) {
    //   double cost = marker.points[i].z;
    //   double scaled_cost = (cost - min_cost)/(max_cost - min_cost);
    //   color.r = scaled_cost;
    //   color.b = 0.5 * (1.0 - scaled_cost);
    //   color.g = 0.0;
    //   color.a = 1.0;
    //   marker.colors.push_back(color);
    //   marker.points[i].z = 0.1 * scaled_cost;

    //   // interchange axes x and z
    //   // double x = marker.points[i].z;
    //   // marker.points[i].z = marker.points[i].x;
    //   // marker.points[i].x = x;
    // }


    // cost_viz_scaling_const_ = min_cost;
    // cost_viz_scaling_factor_ = 0.1 /(max_cost - min_cost);

    rviz_pub_.publish(marker);
  }
}

void StompTest::visualizeTrajectoryStrategy2(
  Rollout& rollout, bool noiseless, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "BASE";
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajectory";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(num_time_steps_);

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.colors.resize(num_time_steps_);
  Eigen::MatrixXd param_sample = Eigen::MatrixXd::Zero(num_dimensions_, 1);
  double cost = 0.0;

  for (int t = 0; t < num_time_steps_; ++t) {
    marker.points[t].x = rollout.parameters_noise_[0][t];
    marker.points[t].y = rollout.parameters_noise_[1][t];
    marker.points[t].z = rollout.parameters_noise_[2][t];

    for (int d = 0; d < num_dimensions_; ++d) {
      param_sample(d, 0) = rollout.parameters_noise_[d][t]; 
    }
    cost = (this->*evaluateStateCostStrategy)(&param_sample);
    if (noiseless) {
      // printf("cost @ time %d is %.3f\n", t, cost); 
    }

    marker.colors[t].g = 1.0 - cost >= 0.0 ? 1.0 - cost : 0.0;
    marker.colors[t].r = cost <= 1.0 ? cost : 1.0;

    if (noiseless) {
      marker.colors[t].a = 1.0; 
    } else {
      marker.colors[t].a = 0.2;
    }
  }

  if (noiseless) {
    marker.scale.x = 0.01;
    // marker.color.a = 1.0;
    // marker.color.r = 0.0;
    // marker.color.g = 1.0;
    // marker.color.b = 0.0;
  } else {
    marker.scale.x = 0.001;
    // marker.color.a = 0.7;
    // marker.color.r = 0.2;
    // marker.color.g = 0.5;
    // marker.color.b = 0.5;
  }

  rviz_pub_.publish(marker);
}

//////////////////////////////////

double StompTest::evaluateStateCostStrategy3(
  Eigen::MatrixXd* param_sample) {
  // param_sample is a joint space q
  // printf("################################\n");
  double cost = 0.0;

  // for map cost, fk it into tool-space, calculate obstacle costs
  // analytic_ur_fk_2(&joints, param_sample, &fk_hom,
  //   &gripper_fixed_hom);

  double x = (*param_sample)(0, 0);
  double y = (*param_sample)(1, 0);
  double z = (*param_sample)(2, 0);

  for (unsigned int o = 0; o < collision_geometries_.size(); ++o) {
    // TODO(jim) add nearest neighbors lookup, fixed size always query n nearest neighbors
    for (unsigned int j = 0; j < obstacle_points_->size(); ++j) {

      double dx = (x - obstacle_points_->at(j)[0])
        / collision_geometries_[o].radius_[0];
      // printf("dx %.3f\n", dx);
      double dy = (y - obstacle_points_->at(j)[1])
        / collision_geometries_[o].radius_[1];
      // printf("dy %.3f\n", dy);
      double dz = (z - obstacle_points_->at(j)[2])
        / collision_geometries_[o].radius_[2];
      // printf("dz %.3f\n", dz);
      double dist = sqrt(dx * dx + dy * dy + dz * dz);
      // printf("dist: %.3f\n", dist);

      // double dx = (fk_hom(0, 3) - obstacle_points_->at(j)[0])
      //   / collision_geometries_[o].radius_[0];
      // double dy = (fk_hom(1, 3) - obstacle_points_->at(j)[1])
      //   / collision_geometries_[o].radius_[1];
      // double dz = (fk_hom(2, 3) - obstacle_points_->at(j)[2])
      //   / collision_geometries_[o].radius_[2];
      // double dist = sqrt(dx * dx + dy * dy + dz * dz);

      // 2020-01-01 semantics:
      // if within the radius from the center
      // of a 'True' obstacle
      // cost is ticked to 1
      // 'inadmissible'

      // if within the radius from the center
      // of a 'False' obstacle
      // if cost is 0.0
      // it is raised to a partial
      // 'admissible' obstacle
      if (collision_geometries_[o].inadmissible_) {
        if (dist < 1.0) {
          // printf("inadmissible collision!!!\n");
          cost += 1.0;
          // cost = 1.0;
          break;
        }
      } else {
        if (dist < 1.0) {
          // printf("admissible collision!!!\n");
          cost += 1.0 - dist;
          // cost = 1.0 - dist;
          break;
        }
      }
    }  
  }

  const double joint_limit_cost = 100.0;
  for (int i = 0; i < joints.size(); ++i) {
    // lower limit, upper limit
    if ((*param_sample)(i, 0) < joints[i].limits[0]) {
      cost += joint_limit_cost * abs((*param_sample)(i, 0) - joints[i].limits[0]);
    }

    // lower limit, upper limit
    if ((*param_sample)(i, 0) > joints[i].limits[1]) {
      cost += joint_limit_cost * abs((*param_sample)(i, 0) - joints[i].limits[1]);
    }
  }

  // printf("COST %.3f\n", cost);

  // for joint costs, impose joint limits (defined in YAML)
  return cost;
}

void StompTest::visualizeCostFunctionStrategy3() {
  printf("visualizeCostFunctionStrategy3\n");
  visualization_msgs::Marker marker;
  marker.header.frame_id = "BASE";
  marker.header.stamp = ros::Time::now();
  marker.ns = "cost";
  marker.id = 0;

  visualization_msgs::Marker points;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(obstacle_points_->size());

  for (int t = 0; t < obstacle_points_->size(); ++t) {
    marker.points[t].x = obstacle_points_->at(t)[0];
    marker.points[t].y = obstacle_points_->at(t)[1];
    marker.points[t].z = obstacle_points_->at(t)[2];
  }

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  rviz_pub_.publish(marker);
}

void StompTest::visualizeTrajectoryStrategy3(
  Rollout& rollout,
  bool noiseless,
  int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "BASE";
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajectory";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(num_time_steps_);

  // marker.colors.resize(num_time_steps_);

  for (int t = 0; t < num_time_steps_; ++t) {
    analytic_ur_fk_3(&joints,
      &rollout.parameters_noise_, t,
      // TODO(jim) rewrite parameters_noise_
      // to eigen matrix
      &fk_hom,
      &gripper_fixed_hom);
    marker.points[t].x = fk_hom(0, 3);
    marker.points[t].y = fk_hom(1, 3);
    marker.points[t].z = fk_hom(2, 3);

    /*
    if (id == 0) {
      // for the noiseless rollout
      // one for current trajectory cost
      // calculation
      printf("fk_hom xyz: %.3f, %.3f, %.3f\n",
        fk_hom(0, 3),
        fk_hom(1, 3),
        fk_hom(2, 3));
    }
    */
  }

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  if (noiseless) {
    marker.scale.x = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  } else {
    marker.scale.x = 0.002;
    marker.color.a = 0.7;
    marker.color.r = 0.2;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  }

  rviz_pub_.publish(marker);
}

void StompTest::saveTrajectoryStrategy3(
  const char* file_abs_path) {
  YAML::Emitter out;

  Rollout noiseless_rollout;
  stomp_->getNoiselessRollout(noiseless_rollout);

  out << YAML::BeginMap;
  out << YAML::Key << "type";
  out << YAML::Value << "CfgTrajectory";

  ///////////////////////////////////////////// params
  out << YAML::Key << "params";
  out << YAML::Value;
  out << YAML::BeginMap;

  ///////////////////////////////////////////// cfgs
  out << YAML::Key << "cfgs";
  out << YAML::Value;
  out << YAML::BeginSeq;  // all cfgs
  for (int t = 0; t < num_time_steps_; ++t) {
    // cfg i
    out << YAML::BeginSeq;
    for (int d = 0; d < num_dimensions_; ++d) {
      out << noiseless_rollout.parameters_noise_[d][t];
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndSeq;  // all cfgs
  /////////////////////////////////////////////

  ///////////////////////////////////////////// cfg_times
  out << YAML::Key << "cfg_times";
  out << YAML::Value;
  out << YAML::BeginSeq;  // all cfg_times
  for (int t = 0; t < num_time_steps_; ++t) {
    out << t * movement_duration_ / num_time_steps_;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;  // cfg_times
  /////////////////////////////////////////////

  out << YAML::EndMap;  // params
  /////////////////////////////////////////////

  std::ofstream fout(file_abs_path);
  fout << out.c_str();
  return;
}

}  // namespace stomp

namespace YAML {

using stomp::yaml::Convert;
using stomp::yaml::ConvertSequence;

bool convert<stomp::Obstacle>::decode(
  const YAML::Node& node,
  stomp::Obstacle& o) {  // NOLINT(runtime/references)
  if (node["center"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::Obstacle requires center component");
  }
  o.center_ = node["center"].as<std::vector<double>>();

  if (node["radius"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::Obstacle requires radius component");
  }
  o.radius_ = node["radius"].as<std::vector<double>>();

  if (node["boolean"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::Obstacle requires boolean component");
  }
  o.inadmissible_ = node["boolean"].as<bool>();

  return true;
}

bool convert<stomp::StompTest>::decode(
  const YAML::Node& node,
  stomp::StompTest& s) {  // NOLINT(runtime/references)
  if (node["num_iterations"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires num_iterations component");
  }
  s.num_iterations_ = node["num_iterations"].as<int>();

  if (node["num_time_steps"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires num_time_steps component");
  }
  s.num_time_steps_ = node["num_time_steps"].as<int>();

  if (node["movement_duration"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires movement_duration component");
  }
  s.movement_duration_ = node["movement_duration"].as<double>();

  if (node["control_cost_weight"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires control_cost_weight component");
  }
  s.control_cost_weight_ = node["control_cost_weight"].as<double>();

  if (node["output_dir"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires output_dir component");
  }
  s.output_dir_ = node["output_dir"].as<std::string>();

  if (node["use_chomp"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires use_chomp component");
  }
  s.use_chomp_ = node["use_chomp"].as<bool>();

  if (node["save_noisy_trajectories"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires save_noisy_trajectories component");
  }
  s.save_noisy_trajectories_ = node["save_noisy_trajectories"].as<bool>();

  if (node["save_noiseless_trajectories"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires save_noiseless_trajectories component");
  }
  s.save_noiseless_trajectories_ =
    node["save_noiseless_trajectories"].as<bool>();

  if (node["save_cost_function"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires save_cost_function component");
  }
  s.save_cost_function_ = node["save_cost_function"].as<bool>();

  if (node["publish_to_rviz"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires publish_to_rviz component");
  }
  s.publish_to_rviz_ = node["publish_to_rviz"].as<bool>();

  if (node["delay_per_iteration"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires delay_per_iteration component");
  }
  s.delay_per_iteration_ = node["delay_per_iteration"].as<double>();

  if (node["params_s"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires params_s component");
  }
  s.params_s_ = node["params_s"].as<std::vector<double>>();
  if (node["params_e"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires params_e component");
  }
  s.params_e_ = node["params_e"].as<std::vector<double>>();

  if (s.params_s_.size() == 0 || s.params_e_.size() == 0) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest found params_s or params_e empty");
  }
  if (s.params_s_.size() != s.params_e_.size()) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest found mismatch of params_s, params_e size");
  }
  // if (node["num_dimensions"] == NULL) {
  //   throw stomp::ExceptionYaml(
  //     "stomp::StompTest requires num_dimensions component");
  // }
  // s.num_dimensions_ = node["num_dimensions"].as<int>();
  // 2019-12-31 TODO(jim) consolidate starting(ending)_point / num_dimensions
  // to infer num_dimensions_ from size of params_s etc.
  s.num_dimensions_ = s.params_s_.size();

  /* load obstacles */
  if (node["collision_geometries"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires obstacles component");
  }
  if (!node["collision_geometries"].IsSequence()) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires obstacles sequence");
  }
  s.collision_geometries_.clear();
  for (unsigned int i = 0; i < node["collision_geometries"].size(); i++) {
    s.collision_geometries_.push_back(
      node["collision_geometries"][i].as<stomp::Obstacle>());
  }

  /* load stomp object */
  if (node["stomp"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires stomp component");
  }
  s.stomp = node["stomp"].as<stomp::STOMP>();

  // 2020-01-03 adding DH joints for fk
  if (node["robot"] == NULL) {
    printf("no robot defined in yaml, ending\n");
    return 1;
  }
  if (node["robot"]["dh_joints"] == NULL) {
    printf("no dh_joints defined in yaml, ending\n");
    return 1;
  }
  std::vector<stomp::DHJoint> joints;
  for (unsigned int i = 0;
    i < node["robot"]["dh_joints"].size(); i++) {
    s.joints.push_back(
      node["robot"]["dh_joints"][i].as<stomp::DHJoint>());
  }
  printf("parsed out %d joints\n", s.joints.size());


  // TODO(jim) parse this from YAML
  std::vector<double> gripper_xyz =
    node["robot"]["gripper_xyz"].as<std::vector<double>>();
  for (int i = 0; i < 3; ++i) {
    s.gripper_fixed_hom(i, 3) = gripper_xyz[i];
  }

  s.resolution_ = node["path_segment_interpolation_resolution"].as<double>();

  /* load func ptrs TODO(jim, maybe) tie this to yaml? */
  s.evaluateStateCostStrategy = &stomp::StompTest::evaluateStateCostStrategy3;
  s.visualizeCostFunctionStrategy =
    &stomp::StompTest::visualizeCostFunctionStrategy3;
  s.visualizeTrajectoryStrategy =
    &stomp::StompTest::visualizeTrajectoryStrategy2;

  return true;
}

}  // namespace YAML
