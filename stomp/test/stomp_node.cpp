/*
 *  Copyright (c) CYAN
 */

#include <string>
#include <vector>

#include <stomp/stomp_test.h>

int main(int argc, char ** argv) {
  int expected_argc = 3;
  if (argc != expected_argc) {
    printf("stomp_node expects argc %d, got %d\n", expected_argc, argc);
  }

  //////// init ops
  ros::init(argc, argv, "stomp_node");

  YAML::Node stomp_test_node = YAML::LoadFile(argv[1]);
  stomp::StompTest stomp_test = stomp_test_node.as<stomp::StompTest>();

  YAML::Node obstacle_points_node = YAML::LoadFile(argv[2]);
  std::vector<std::vector<double>> obstacle_points
    = obstacle_points_node["points"].as<
      std::vector<std::vector<double>>>();
  printf("found %d obstacles\n",
    obstacle_points.size());

  // /* evaluate the cost of the initial_trajectory against the obstacles */
  // std::vector<Eigen::VectorXd> initial_trajectory;
  // initial_trajectory.resize(num_dimensions_,
  //   Eigen::VectorXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING));

  // std::vector<Eigen::MatrixXd> derivative_costs;
  // derivative_costs.resize(
  //   num_dimensions_,
  //   Eigen::MatrixXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING,
  //     NUM_DIFF_RULES));

  // for (int d = 0; d < num_dimensions_; ++d) {
  //   // apply starting point and ending point here
  //   initial_trajectory[d].head(TRAJECTORY_PADDING) =
  //     params_s_[d]*Eigen::VectorXd::Ones(
  //     TRAJECTORY_PADDING);
  //   initial_trajectory[d].tail(TRAJECTORY_PADDING) =
  //     params_e_[d]*Eigen::VectorXd::Ones(
  //     TRAJECTORY_PADDING);

  //   derivative_costs[d].col(STOMP_ACCELERATION) = Eigen::VectorXd::Ones(
  //     num_time_steps_ + 2*TRAJECTORY_PADDING);
  // }


  // need this otherwise breaks enable_shared_from_this
  boost::shared_ptr<stomp::StompTest> test(&stomp_test,
    &stomp::null_deleter<stomp::StompTest>);

  int res = test->run(&obstacle_points);

  return res;
}
