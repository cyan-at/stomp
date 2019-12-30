/*
 * stomp_2d_00.cpp
 *  Copyright (c) CYAN
 *
 *  2019-12-22, standalone STOMP demo program
 #  No ROS involvement, only writes to disk
 */

#include <string>
#include <vector>

struct Obstacle2D {
  std::vector<double> center_;
  std::vector<double> radius_;
  bool boolean_;
};

int main(int argc, char ** argv) {
  srand(time(NULL));
  return 0;
}
