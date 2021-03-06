/*
 * cartesian_orientation_feature.h
 *
 *  Created on: May 30, 2012
 *      Author: kalakris
 */

#ifndef CARTESIAN_ORIENTATION_FEATURE_H_
#define CARTESIAN_ORIENTATION_FEATURE_H_

#include <stomp_ros_interface/cost_features/stomp_cost_feature.h>

namespace stomp_ros_interface
{

class CartesianOrientationFeature: public StompCostFeature
{
public:
  CartesianOrientationFeature();
  virtual ~CartesianOrientationFeature();

  virtual bool initialize(XmlRpc::XmlRpcValue& config, const StompRobotModel::StompPlanningGroup* planning_group);
  virtual int getNumValues() const;
  virtual void computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> input, std::vector<double>& feature_values,
                                         bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity);
  virtual std::string getName() const;
  virtual boost::shared_ptr<learnable_cost_function::Feature> clone() const;

  void getNames(std::vector<std::string>& names) const;

};

} /* namespace stomp_ros_interface */
#endif /* CARTESIAN_ORIENTATION_FEATURE_H_ */
