/*
 * stomp_2d_test.h
 *  Copyright (c) 2010, Willow Garage, Inc.
 *
 *  Created on: Feb 2, 2012
 *      Author: kalakris
 */

#ifndef STOMP_TEST_STOMP_2D_TEST_H_
#define STOMP_TEST_STOMP_2D_TEST_H_

#include <stomp/stomp.h>
#include <stomp/ur_kin.h>
// #include <stomp/chomp.h>
#include <stomp/task.h>
#include <yaml-cpp/yaml.h>

#include "stomp/HandlePlanStompSrv.h"

// 2020-01-06 timing difference
#include <time.h>

#include <stomp/stomp_utils.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <vector>
#include <fstream>

#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sstream>
#include <cstdio>

#include <boost/enable_shared_from_this.hpp>

namespace stomp {

class Obstacle {
 public:
  std::vector<double> center_;
  std::vector<double> radius_;
  bool inadmissible_;

  Obstacle() {}
};

class StompTest: public Task,
  public boost::enable_shared_from_this<StompTest> {
 public:
  StompTest():
    node_handle_("test_stomp2d"), initialized_(false) {
      // ros interfaces
      // TODO(jim) remove these maybe
      rviz_pub_ = node_handle_.advertise<visualization_msgs::Marker>(
        "visualization", 100, false);
      ros::ServiceServer service = node_handle_.advertiseService(
        "plan_stomp", &StompTest::HandlePlanStomp, this);
    }

  int run(std::vector<std::vector<double>>* obstacle_points);

  // functions inherited from Task:

  /**
   * Initialize the task for a given number of threads.
   * @param num_threads Number of threads for multi-threading
   * @return
   */
  virtual bool initialize(int num_threads, int num_rollouts);
  bool initialized_;

  /**
   * Executes the task for the given policy parameters, and returns the costs per timestep
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
   * @param weighted_feature_values num_time_steps x num_features matrix of weighted feature values per time step
   * @return
   */
  virtual bool execute(
    const std::vector<Eigen::VectorXd>& parameters,
    const std::vector<Eigen::VectorXd>& projected_parameters,
    Eigen::VectorXd* costs,
    Eigen::MatrixXd* weighted_feature_values,
    const int iteration_number,
    const int rollout_number,
    int thread_id,
    bool compute_gradients,
    std::vector<Eigen::VectorXd>& gradients,
    bool& validity);

  virtual bool filter(
    std::vector<Eigen::VectorXd>& parameters, int thread_id) const;

  /**
   * Get the Policy object of this Task
   * @param policy
   * @return
   */
  virtual bool getPolicy(
    boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy);

  /**
   * Sets the Policy object of this Task
   * @param policy
   * @return
   */
  virtual bool setPolicy(
    const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy);

  /**
   * Gets the weight of the control cost
   * @param control_cost_weight
   * @return
   */
  virtual double getControlCostWeight();

 // private:
  stomp::STOMP stomp;
  boost::shared_ptr<stomp::STOMP> stomp_;
  // boost::shared_ptr<stomp::CHOMP> chomp_;
  boost::shared_ptr<stomp::CovariantMovementPrimitive> policy_;
  ros::NodeHandle node_handle_;
  ros::Publisher rviz_pub_;
  bool publish_to_rviz_;

  int num_iterations_;
  int num_time_steps_;
  int num_dimensions_;
  double movement_duration_;
  double movement_dt_;
  double control_cost_weight_;
  std::string output_dir_;
  bool use_chomp_;
  bool save_noisy_trajectories_;
  bool save_noiseless_trajectories_;
  bool save_cost_function_;
  double resolution_;
  double delay_per_iteration_;
  double cost_viz_scaling_const_;
  double cost_viz_scaling_factor_;

  std::vector<Obstacle> collision_geometries_;
  std::vector<std::vector<double>>* obstacle_points_;

  Eigen::MatrixXd vel_diff_matrix_;
  Eigen::MatrixXd acc_diff_matrix_;

  std::vector<double> params_s_;
  std::vector<double> params_e_;

  std::vector<Eigen::VectorXd> initial_trajectory_;

  double interpPathSegmentAndEvaluateStateCost(
    Eigen::MatrixXd* last_param_sample,
    Eigen::MatrixXd* this_param_sample,
    double vx, double vy,
    bool compute_gradients,
    double ax, double ay,
    double& gx, double& gy);
  // calls evaluateStateCostWithGradients

  double evaluateStateCostWithGradients(
    Eigen::MatrixXd* param_sample,
    double vx, double vy,
    bool compute_gradients,
    double ax, double ay,
    double& gx, double& gy);
  // calls evaluateStateCostStrategy*, evaluateStateCostGradientStrategy1

  bool HandlePlanStomp(
    HandlePlanStompSrv::Request& req,    // NOLINT(runtime/references)
    HandlePlanStompSrv::Response& res);  // NOLINT(runtime/references)

  ////////////////////////////////// STRATEGY FUNCPTR DEFINITIONS

  double (StompTest::*evaluateStateCostStrategy)(
    Eigen::MatrixXd* param_sample);

  void (StompTest::*visualizeCostFunctionStrategy)();

  void (StompTest::*visualizeTrajectoryStrategy)(
    Rollout& rollout,
    bool noiseless,
    int id);

  void (StompTest::*saveTrajectoryStrategy)(
    std::string* file_abs_path);

  ////////////////////////////////// STRATEGY SETS

  //////////////////////////////////

  double evaluateStateCostStrategy1(
    Eigen::MatrixXd* param_sample);
  void evaluateStateCostGradientStrategy1(
    Eigen::MatrixXd* param_sample,
    double& gx, double& gy);

  void visualizeCostFunctionStrategy1();
  void visualizeTrajectoryStrategy1(
    Rollout& rollout,
    bool noiseless,
    int id);

  //////////////////////////////////

  double evaluateStateCostStrategy2(
    Eigen::MatrixXd* param_sample);

  void visualizeCostFunctionStrategy2();
  void visualizeTrajectoryStrategy2(
    Rollout& rollout,
    bool noiseless,
    int id);

  //////////////////////////////////

  std::vector<stomp::DHJoint> joints;
  Hom fk_hom = Eigen::MatrixXd::Identity(4, 4);

  Hom gripper_fixed_hom = Eigen::MatrixXd::Identity(4, 4);

  double evaluateStateCostStrategy3(
    Eigen::MatrixXd* param_sample);

  void visualizeCostFunctionStrategy3();
  void visualizeTrajectoryStrategy3(
    Rollout& rollout,
    bool noiseless,
    int id);

  void saveTrajectoryStrategy3(
    const char* file_abs_path);
};

}  // namespace stomp

namespace YAML {

template <>
struct convert<stomp::Obstacle> {
  static bool decode(const YAML::Node& node,
    stomp::Obstacle& o);  // NOLINT(runtime/references)
};

template <>
struct convert<stomp::StompTest> {
  static bool decode(const YAML::Node& node,
    stomp::StompTest& s);  // NOLINT(runtime/references)
};

}  // namespace YAML

#endif  // STOMP_TEST_STOMP_2D_TEST_H_
