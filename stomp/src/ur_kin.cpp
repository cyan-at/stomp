// Unpublished Copyright (c)

#include <stomp/ur_kin.h>

namespace stomp {


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
  joint.alpha = node["alpha"].as<double>();

  if (node["limits"] == NULL) {
    throw stomp::ExceptionYaml(
      "stomp::DHJoint requires limits component");
  }
  joint.limits = node["limits"].as<std::vector<double>>();

  return true;
}

}  // namespace YAML