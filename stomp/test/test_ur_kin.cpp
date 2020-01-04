/*
 *  Copyright (c)
 *
*/

#include <iostream>

#include <stomp/ur_kin.h>

// ##################################################################

int main(int argc, char ** argv) {
  if (argc != 3) {
    printf("expecting 2 args\n");
    return 1;
  }

  YAML::Node stomp_yaml_node = YAML::LoadFile(argv[1]);
  if (stomp_yaml_node["robot"] == NULL) {
    printf("no robot defined in yaml, ending\n");
    return 1;
  }

  if (stomp_yaml_node["robot"]["dh_joints"] == NULL) {
    printf("no dh_joints defined in yaml, ending\n");
    return 1;
  }

  std::vector<stomp::DHJoint> joints;
  for (unsigned int i = 0;
    i < stomp_yaml_node["robot"]["dh_joints"].size(); i++) {
    joints.push_back(
      stomp_yaml_node["robot"]["dh_joints"][i].as<stomp::DHJoint>());
  }

  ////////////////////////////////////////////////////////////////////////////////////

  std::vector<std::vector<double>> cfgs;
  // some ad-hoc convert, decoding
  YAML::Node cfg_yaml_node = YAML::LoadFile(argv[2]);
  if (cfg_yaml_node["params"] == NULL) {
    printf("no params defined in yaml, ending\n");
    return 1;
  }
  if (cfg_yaml_node["params"]["cfgs"] == NULL) {
    printf("no params/cfgs defined in yaml, ending\n");
    return 1;
  }
  if (!cfg_yaml_node["params"]["cfgs"].IsSequence()) {
    printf("no params/cfgs sequence defined in yaml, ending\n");
    return 1;
  }
  for (unsigned int i = 0; i < cfg_yaml_node["params"]["cfgs"].size(); i++) {
    cfgs.push_back(cfg_yaml_node["params"]["cfgs"][i].as<std::vector<double>>());
  }

  stomp::Hom fk_hom = Eigen::MatrixXd::Identity(4, 4);
  for (int cfg_i = 0; cfg_i < cfgs.size(); ++ cfg_i) {
    analytic_ur_fk(&joints, &cfgs.at(cfg_i), &fk_hom);
    printf("fk_hom\n");
    std::cout << fk_hom << std::endl;
  }

  return 0;
}