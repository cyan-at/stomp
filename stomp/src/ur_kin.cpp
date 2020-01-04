// Unpublished Copyright (c)

#include <stomp/ur_kin.h>

namespace stomp {

void DHJoint::calc_fk(double q) {
  // TODO(jim) rewrite q to Eigen vector
  sq_ = sin(q);
  cq_ = cos(q);

  a_matrix(0, 0) = cq_;
  a_matrix(0, 1) = -sq_ * calpha_;
  a_matrix(0, 2) = sq_ * salpha_;
  a_matrix(0, 3) = r_a * cq_;

  a_matrix(1, 0) = sq_;
  a_matrix(1, 1) = cq_ * calpha_;
  a_matrix(1, 2) = -cq_ * salpha_;
  a_matrix(1, 3) = r_a * sq_;

  a_matrix(2, 1) = salpha_;
  a_matrix(2, 2) = calpha_;
  a_matrix(2, 3) = d;
}

}  // namespace stomp


namespace YAML {

using stomp::yaml::Convert;
using stomp::yaml::ConvertSequence;

bool convert<stomp::DHJoint>::decode(
  const YAML::Node& node,
  stomp::DHJoint& joint) {  // NOLINT(runtime/references)
  if (node["d"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::DHJoint requires d component");
  }
  joint.d = node["d"].as<double>();

  if (node["r_a"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::DHJoint requires r_a component");
  }
  joint.r_a = node["r_a"].as<double>();

  if (node["alpha"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::DHJoint requires alpha component");
  }
  joint.set_alpha(node["alpha"].as<double>());

  if (node["limits"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::DHJoint requires limits component");
  }
  joint.limits = node["limits"].as<std::vector<double>>();

  // default joint angle
  joint.calc_fk(0.0);

  return true;
}

}  // namespace YAML