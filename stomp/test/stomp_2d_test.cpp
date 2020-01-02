/*
 * stomp_2d_test.cpp
 *  Copyright (c) 2010, Willow Garage, Inc.
 *
 *  Created on: Feb 2, 2012
 *      Author: kalakris
 */

#include "./stomp_2d_test.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <time.h>
#include <stomp/stomp_utils.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <sstream>
#include <cstdio>

namespace stomp {

int StompTest::run() {
  stomp::DHJoint ur10_params;
  ur10_params.d1 = 0.1273;
  ur10_params.d4 = 0.163941;
  ur10_params.d5 = 0.1157;
  ur10_params.d6 = 0.0922;
  ur10_params.a2 = -0.612;
  ur10_params.a3 = -0.5723;

  // initialize rviz publisher
  rviz_pub_ = node_handle_.advertise<visualization_msgs::Marker>(
    "visualization", 100, false);
  srand(time(NULL));
  resolution_ = 0.002;
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

  std::vector<Eigen::MatrixXd> derivative_costs;
  std::vector<Eigen::VectorXd> initial_trajectory;
  derivative_costs.resize(num_dimensions_, Eigen::MatrixXd::Zero(
    num_time_steps_ + 2*TRAJECTORY_PADDING, NUM_DIFF_RULES));
  initial_trajectory.resize(num_dimensions_, Eigen::VectorXd::Zero(
    num_time_steps_ + 2*TRAJECTORY_PADDING));

  for (int d = 0; d < num_dimensions_; ++d) {
    // apply starting point and ending point here
    initial_trajectory[d].head(TRAJECTORY_PADDING) =
      starting_point_[d]*Eigen::VectorXd::Ones(
      TRAJECTORY_PADDING);
    initial_trajectory[d].tail(TRAJECTORY_PADDING) =
      ending_point_[d]*Eigen::VectorXd::Ones(
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

    if (publish_to_rviz_) {
      // wait until delay_per_iteration
      double delay = 0.0;
      while (delay < delay_per_iteration_) {
        delay = (ros::Time::now() - prev_iter_stamp).toSec();
        if (delay < delay_per_iteration_) {
          ros::Duration(delay_per_iteration_ - delay).sleep();
        }
      }
      prev_iter_stamp = ros::Time::now();

      (this->*visualizeTrajectoryStrategy)(noiseless_rollout, true, 0);
      if (!use_chomp_) {
        for (size_t j=0; j < rollouts.size(); ++j) {
          (this->*visualizeTrajectoryStrategy)(rollouts[j], false, j+1);
        }
      }
    }
  }

  fclose(stddev_file);
  fclose(cost_file);
  if (save_noisy_trajectories_)
    fclose(num_rollouts_file);

  stomp_.reset();
  // chomp_.reset();
  policy_.reset();

  return 0;
}

bool StompTest::initialize(int num_threads, int num_rollouts) {
  return true;
}

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

  // 2020-01-01 instead of px / py, rewrite to scalable
  // get the t'th column of pos
  double px = 0.01;
  double py = 0.01;
  Eigen::MatrixXd last_param_sample(num_dimensions_, 1);
  last_param_sample = pos.col(0);

  Eigen::MatrixXd this_param_sample(num_dimensions_, 1);

  for (int t = TRAJECTORY_PADDING;
    t < TRAJECTORY_PADDING+num_time_steps_; ++t) {
    // compute the cost for this rollout

    double x = pos(0, t);
    double y = pos(1, t);
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

    px = x;
    py = y;
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
  double& gx, double& gy) const {

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
  double& gx, double& gy) const {
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
  Eigen::MatrixXd* param_sample) const {
  double cost = 0.0;

  double x = (*param_sample)(0, 0);
  double y = (*param_sample)(1, 0);

  for (unsigned int o = 0; o < obstacles_.size(); ++o) {
    double dx = (x - obstacles_[o].center_[0])
      / obstacles_[o].radius_[0];
    double dy = (y - obstacles_[o].center_[1])
      / obstacles_[o].radius_[1];

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
    if (obstacles_[o].inadmissible_) {
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
  double& gx, double& gy) const {
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
  Eigen::MatrixXd* param_sample) const {
  double cost = 0.0;

  double x = (*param_sample)(0, 0);
  double y = (*param_sample)(1, 0);
  double z = (*param_sample)(2, 0);

  for (unsigned int o = 0; o < obstacles_.size(); ++o) {
    double dx = (x - obstacles_[o].center_[0])
      / obstacles_[o].radius_[0];
    double dy = (y - obstacles_[o].center_[1])
      / obstacles_[o].radius_[1];
    double dz = (z - obstacles_[o].center_[2])
      / obstacles_[o].radius_[2];
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
    if (obstacles_[o].inadmissible_) {
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
  for (int obstacle_i = 0; obstacle_i < obstacles_.size(); ++obstacle_i) {
    visualization_msgs::Marker marker;

    marker.id = obstacle_i;
    marker.ns = "cost";
    marker.header.frame_id = "BASE";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = obstacles_[obstacle_i].center_[0];
    marker.pose.position.y = obstacles_[obstacle_i].center_[1];
    marker.pose.position.z = obstacles_[obstacle_i].center_[2];

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = obstacles_[obstacle_i].radius_[0];
    marker.scale.y = obstacles_[obstacle_i].radius_[1];
    marker.scale.z = obstacles_[obstacle_i].radius_[2];

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

  // marker.colors.resize(num_time_steps_);

  Eigen::MatrixXd param_sample = Eigen::MatrixXd::Zero(num_dimensions_, 1);
  for (int t = 0; t < num_time_steps_; ++t) {
    marker.points[t].x = rollout.parameters_noise_[0][t];
    marker.points[t].y = rollout.parameters_noise_[1][t];
    marker.points[t].z = rollout.parameters_noise_[2][t];
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

  if (node["starting_point"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires starting_point component");
  }
  s.starting_point_ = node["starting_point"].as<std::vector<double>>();
  if (node["ending_point"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires ending_point component");
  }
  s.ending_point_ = node["ending_point"].as<std::vector<double>>();

  if (s.starting_point_.size() == 0 || s.ending_point_.size() == 0) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest found starting_point or ending_point empty");
  }
  if (s.starting_point_.size() != s.ending_point_.size()) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest found mismatch of starting_point, ending_point size");
  }
  // if (node["num_dimensions"] == NULL) {
  //   throw stomp::ExceptionYaml(
  //     "stomp::StompTest requires num_dimensions component");
  // }
  // s.num_dimensions_ = node["num_dimensions"].as<int>();
  // 2019-12-31 TODO(jim) consolidate starting(ending)_point / num_dimensions
  // to infer num_dimensions_ from size of starting_point etc.
  s.num_dimensions_ = s.starting_point_.size();

  /* load obstacles */
  if (node["obstacles"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires obstacles component");
  }
  if (!node["obstacles"].IsSequence()) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires obstacles sequence");
  }
  s.obstacles_.clear();
  for (unsigned int i = 0; i < node["obstacles"].size(); i++) {
    s.obstacles_.push_back(
      node["obstacles"][i].as<stomp::Obstacle>());
  }

  /* load stomp object */
  if (node["stomp"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::StompTest requires stomp component");
  }
  s.stomp = node["stomp"].as<stomp::STOMP>();

  /* load func ptrs TODO(jim, maybe) tie this to yaml? */
  s.evaluateStateCostStrategy = &stomp::StompTest::evaluateStateCostStrategy2;
  s.visualizeCostFunctionStrategy = &stomp::StompTest::visualizeCostFunctionStrategy2;
  s.visualizeTrajectoryStrategy = &stomp::StompTest::visualizeTrajectoryStrategy2;

  return true;
}

}  // namespace YAML

// ##################################################################

int main(int argc, char ** argv) {
  ros::init(argc, argv, "test_stomp2d");

  YAML::Node n = YAML::LoadFile(argv[1]);
  #ifdef DEBUG_VERBOSE
  std::cout << "loaded n" << std::endl;
  std::cout << n << std::endl;
  #endif

  stomp::StompTest stomp_test = n.as<stomp::StompTest>();

  // need this otherwise breaks enable_shared_from_this
  boost::shared_ptr<stomp::StompTest> test(&stomp_test,
    &stomp::null_deleter<stomp::StompTest>);
  int res = test->run();

  return res;
}
