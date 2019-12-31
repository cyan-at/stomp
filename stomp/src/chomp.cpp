/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <stomp/chomp.h>

namespace stomp {

CHOMP::CHOMP() {}

CHOMP::~CHOMP() {}

bool CHOMP::initialize(
  ros::NodeHandle& node_handle, boost::shared_ptr<Task> task) {
  node_handle_ = node_handle;
  STOMP_VERIFY(readParameters());

  task_ = task;
  task_->getPolicy(policy_);
  policy_->getNumTimeSteps(num_time_steps_);
  control_cost_weight_ = task_->getControlCostWeight();
  policy_->getNumDimensions(num_dimensions_);
  policy_->getControlCosts(control_costs_);
  policy_->getInvControlCosts(inv_control_costs_);
  policy_->getParameters(parameters_);
  update_.resize(num_dimensions_, Eigen::VectorXd(num_time_steps_));
  noiseless_rollout_.noise_.resize(
    num_time_steps_, Eigen::VectorXd::Zero(num_time_steps_));
  noiseless_rollout_.control_costs_.resize(
    num_time_steps_, Eigen::VectorXd::Zero(num_time_steps_));
  return true;
}

bool CHOMP::runSingleIteration(int iteration_number) {
  policy_->getParameters(parameters_);
  bool validity = false;
  task_->execute(
    parameters_,
    parameters_,
    &noiseless_rollout_.state_costs_,
    &weighted_feature_values, iteration_number, -1,
    0, true, gradients_, validity);
  policy_->computeControlCosts(
    parameters_, noiseless_rollout_.noise_,
    control_cost_weight_, noiseless_rollout_.control_costs_);
  policy_->computeControlCostGradient(parameters_,
    control_cost_weight_, control_cost_gradients_);

  noiseless_rollout_.parameters_ = parameters_;
  noiseless_rollout_.total_cost_ = noiseless_rollout_.state_costs_.sum();
  for (int d=0; d<num_dimensions_; ++d) {
    noiseless_rollout_.total_cost_ += noiseless_rollout_.control_costs_[d].sum();
  }

  ROS_INFO("Cost = %f", noiseless_rollout_.total_cost_);

  for (int d=0; d<num_dimensions_; ++d) {
    //std::cout << "Dimension " << d << "gradient = \n" << (gradients_[d] + control_cost_gradients_[d]);
    update_[d] = -learning_rate_ * inv_control_costs_[d] * (gradients_[d] + control_cost_gradients_[d]);
    // scale the update
    double max = update_[d].array().abs().matrix().maxCoeff();
    if (max > max_update_)
    {
      update_[d] *= max_update_ / max;
    }
    //std::cout << "Dimension " << d << "update = \n" << update_[d];
    parameters_[d] += update_[d];
  }
  policy_->setParameters(parameters_);

  return true;
}

void CHOMP::getNoiselessRollout(Rollout& rollout) {
  rollout = noiseless_rollout_;
}

bool CHOMP::readParameters() {
  STOMP_VERIFY(node_handle_.getParam("learning_rate", learning_rate_));
  STOMP_VERIFY(node_handle_.getParam("max_update", max_update_));
  return true;
}

} /* namespace stomp */
