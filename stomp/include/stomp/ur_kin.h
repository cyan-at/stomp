// Unpublished Copyright (c)

#ifndef SRC_STOMP_INCLUDE_STOMP_UR_KIN_H_
#define SRC_STOMP_INCLUDE_STOMP_UR_KIN_H_

#include <stomp/stomp_utils.h>

#include <math.h>

#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>
#include <limits>
#include <vector>

#define ZERO_THRESH 1e-8
#define kNumJointsInArm 6

#define QNEAR_MODE_0 0
#define QNEAR_MODE_1 1

namespace stomp {

class DHJoint {
 private:
  // used during calc_fk
  double sq_, cq_, salpha_, calpha_;

 public:
  double d, r_a, alpha;
  std::vector<double> limits;

  Hom a_matrix;

  DHJoint() : d(0.0), r_a(0.0), alpha(0.0) {
    a_matrix = Eigen::MatrixXd::Identity(4, 4);
  }

  void set_alpha(double new_alpha) {
    alpha = new_alpha;
    salpha_ = sin(new_alpha);
    calpha_ = cos(new_alpha);
  }

  void calc_fk(double q);
  // updates a_matrix
};

int SIGN(double x) {
  return (x > 0) - (x < 0);
}

double qnear_err_func_0(double* qnear, double* solution_qs,
  double* min_costs, double* min_vals) {
  double cost = 0.0;
  // printf("qs: ");
  for (int i = 0; i < kNumJointsInArm; ++i) {
    // printf("%.3f ", solution_qs[i]);
    cost += fabs(qnear[i] - solution_qs[i]);
  }
  // printf("\n");
  return cost;
}

void smallest_diff_with_multiples(double* a, double* b,
  double delta_multiple_on_b, int increment,
  double* smallest_diff,
  int* up_m,
  double* last_valid_b_val) {
  // port from wall_borg.interaction.utils.py
  // for example comparing two joint angles where
  // increment/decrement of 2*pi is semantically the same
  // but numerically closer and also a different traj
  // moves joint a different directions
  *up_m = 0;

  *smallest_diff = fabs(*a - (*b + (*up_m) * delta_multiple_on_b));

  *up_m += increment;
  double smallest_diff_candidate = fabs(*a -
    (*b + (*up_m) * delta_multiple_on_b));
  // assumes dips once on the first += increment
  // otherwise bails immediately it will only get worse

  // keep hunting for smallest_diff, stop when you overshoot
  while (smallest_diff_candidate < (*smallest_diff)) {
    *smallest_diff = smallest_diff_candidate;

    *up_m += increment;
    smallest_diff_candidate = fabs(*a -
      (*b + (*up_m) * delta_multiple_on_b));
  }

  // the last valid b value incremented by delta_multiple_on_b
  // is the value of b before you overshot
  *last_valid_b_val = (*b + (
    *up_m - increment) * delta_multiple_on_b);
}

double qnear_err_func_1(double* qnear, double* solution_qs,
  double* min_costs, double* min_vals) {
  double* neg_costs = new double[kNumJointsInArm];
  double* neg_vals = new double[kNumJointsInArm];
  for (int i = 0; i < kNumJointsInArm; ++i) {
    int up_m;
    smallest_diff_with_multiples(
      &qnear[i], &solution_qs[i], 2*M_PI, -1,
      &neg_costs[i],
      &up_m,
      &neg_vals[i]);
  }

  double* pos_costs = new double[kNumJointsInArm];
  double* pos_vals = new double[kNumJointsInArm];
  for (int i = 0; i < kNumJointsInArm; ++i) {
    int up_m;
    smallest_diff_with_multiples(
      &qnear[i], &solution_qs[i], 2*M_PI, 1,
      &pos_costs[i],
      &up_m,
      &pos_vals[i]);
  }

  double min_cost = 0.0;
  for (int i = 0; i < kNumJointsInArm; ++i) {
    min_costs[i] = (neg_costs[i] < pos_costs[i]) ? neg_costs[i] : pos_costs[i];
    min_cost += min_costs[i];
    min_vals[i] = (neg_costs[i] < pos_costs[i]) ? neg_vals[i] : pos_vals[i];
  }

  delete[] neg_costs;
  delete[] neg_vals;
  delete[] pos_costs;
  delete[] pos_vals;

  return min_cost;
}

typedef double (*qnear_func)(double* qnear, double* solution_qs,
  double* min_costs, double* min_vals);

/*
int analytic_ur_ik(DHJoint* params,
  const Hom& hom,
  // 2019-03-22 use const ref, not pointer
  // pointer will cause segfault in SWIG wrapped python
  double* q_sols, int n,
  // 2019-03-22
  // n is un-used, but necessary to use np.array python
  // with %apply in dynamics.i to pass np.array as a double*
  double q6_des,
  // 2019-03-22
  // qnear is specified, qnear_mode QNEAR_MODE_0
  // qnear is current q, qnear_mode QNEAR_MODE_1
  double* qnear, int m,
  // m is un-used, but necessary to use np.array python
  int qnear_mode,
  int* num_sols, int o,
  // o is un-used, but necessary to use np.array python
  double* qnear_best_sol, int p
  // p is un-used, but necessary to use np.array python
  // 2019-03-26 qnear_best_sol will be left untouched
  // if qnear_mode is QNEAR_MODE_0
  ) {
  *num_sols = 0;

  // get out values
  double T00 = (hom)(0, 0);
  double T01 = (hom)(0, 1);
  double T02 = (hom)(0, 2);
  double T03 = (hom)(0, 3);

  double T10 = (hom)(1, 0);
  double T11 = (hom)(1, 1);
  double T12 = (hom)(1, 2);
  double T13 = (hom)(1, 3);

  double T20 = (hom)(2, 0);
  double T21 = (hom)(2, 1);
  double T22 = (hom)(2, 2);
  double T23 = (hom)(2, 3);

  // solve shoulder rotate joint (q1)
  double q1[2];

  double A = params->d6*T12 - T13;
  // std::cout << std::setprecision(7) << "d6 " << d6
  //  << ", a_y " << T12 << ", p_y " << T13 << std::endl;

  double B = params->d6*T02 - T03;
  double R = A*A + B*B;

  // std::cout << std::setprecision(7) << "p_0_05_y " << A
  //  << ", p_0_05_x " << B << ", r " << R << std::endl;

  if (fabs(A) < ZERO_THRESH) {
    double div;
    if (fabs(fabs(params->d4) - fabs(B)) < ZERO_THRESH)
      div = -SIGN(params->d4)*SIGN(B);
    else
      div = -params->d4/B;
    double arcsin = asin(div);
    if (fabs(arcsin) < ZERO_THRESH)
      arcsin = 0.0;
    if (arcsin < 0.0)
      q1[0] = arcsin + 2.0*M_PI;
    else
      q1[0] = arcsin;
    q1[1] = M_PI - arcsin;
  } else if (fabs(B) < ZERO_THRESH) {
    double div;
    if (fabs(fabs(params->d4) - fabs(A)) < ZERO_THRESH)
      div = SIGN(params->d4)*SIGN(A);
    else
      div = params->d4/A;
    double arccos = acos(div);
    q1[0] = arccos;
    q1[1] = 2.0*M_PI - arccos;
  } else if (params->d4*params->d4 > R) {
    return -1;
  } else {
    double arccos = acos(params->d4 / sqrt(R));
    double arctan = atan2(-B, A);

    double pos = arccos + arctan;
    if (fabs(pos) < ZERO_THRESH)
      pos = 0.0;
    if (pos >= 0.0) {
      q1[0] = pos;
    } else {
      q1[0] = 2.0*M_PI + pos;
    }

    double neg = -arccos + arctan;
    if (fabs(neg) < ZERO_THRESH) {
      neg = 0.0;
    }
    if (neg >= 0.0) {
      q1[1] = neg;
    } else {
      q1[1] = 2.0*M_PI + neg;
    }

    // std::cout << std::setprecision(7) << "arctan "
    //  << arctan << ", arccos " << arccos
    //  << ", pos " << pos << ", neg " << neg << std::endl;
  }

  // solve wrist 2 joint (q5)
  double q5[2][2];
  for (int i = 0; i < 2; ++i) {
    // std::cout << "q0_candidate: " << q1[i] << std::endl;
    double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-params->d4);
    double div;
    // std::cout << "d6 " << d6 << std::endl;
    // std::cout << "numer: " << numer << " "
    //   << fabs(fabs(numer) - fabs(d6)) << std::endl;
    if (fabs(fabs(numer) - fabs(params->d6)) < ZERO_THRESH) {
      // std::cout << "<" << std::endl;
      div = SIGN(numer) * SIGN(params->d6);
    } else {
      // std::cout << ">=" << std::endl;
      div = numer / params->d6;
    }
    // std::cout << "div: " << div << std::endl;
    double arccos = acos(div);
    q5[i][0] = arccos;
    q5[i][1] = 2.0*M_PI - arccos;
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      // std::cout << std::setprecision(5) << "\n\nq0 "
      //   << q1[i] << " " << "and q4 " << q5[i][j] << std::endl;
      double c1 = cos(q1[i]), s1 = sin(q1[i]);
      double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);

      // solve wrist 3 joint (q6) //////////////////////////////
      double q6;
      if (fabs(s5) < ZERO_THRESH) {
        q6 = q6_des;
      } else {
        double atan_y = SIGN(s5)*-(T01*s1 - T11*c1);
        double atan_x = SIGN(s5)*(T00*s1 - T10*c1);

        // std::cout << "=====================================" << std::endl;
        // std::cout << std::setprecision(5) << "q1[i]: " << q1[i] << std::endl;
        // std::cout << std::setprecision(5) << "s5: " << s5 << std::endl;
        // std::cout << std::setprecision(5) << "c1: " << c1 << std::endl;
        // std::cout << std::setprecision(5) << "s1: " << s1 << std::endl;
        // std::cout << std::setprecision(3) << "o_x: " << T01 << std::endl;
        // std::cout << std::setprecision(3) << "o_y: " << T11 << std::endl;
        // std::cout << std::setprecision(3) << "n_x: " << T00 << std::endl;
        // std::cout << std::setprecision(3) << "n_y: " << T10 << std::endl;

        // std::cout << std::setprecision(3)
        //   << "atan_y: " << atan_y << std::endl;
        // std::cout << std::setprecision(3)
        // << "atan_x: " << atan_x << std::endl;

        q6 = atan2(atan_y, atan_x);
        if (fabs(q6) < ZERO_THRESH) {
          q6 = 0.0;
        }
        if (q6 < 0.0) {
          q6 += 2.0*M_PI;
        }
      }
      // std::cout << "q6: " << q6 << std::endl;

      // solv RRR joints (q2, q3, q4)
      double q2[2], q3[2], q4[2];
      double c6 = cos(q6), s6 = sin(q6);
      double x04x = -s5*(T02*c1 + T12*s1)
        - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
      double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
      double p13x = params->d5*(s6*(T00*c1 + T10*s1)
        + c6*(T01*c1 + T11*s1)) - params->d6*(T02*c1 + T12*s1)
        + T03*c1 + T13*s1;
      double p13y = T23 - params->d1 - params->d6*T22
        + params->d5*(T21*c6 + T20*s6);

      double c3 = (p13x*p13x + p13y*p13y - pow(params->a2, 2)
        - pow(params->a3, 2)) / (2.0*params->a2*params->a3);
      if (fabs(fabs(c3) - 1.0) < ZERO_THRESH) {
        c3 = SIGN(c3);
      } else if (fabs(c3) > 1.0) {
        // NO SOLUTION
        continue;
      }
      double arccos = acos(c3);
      q3[0] = arccos;
      q3[1] = 2.0 * M_PI - arccos;
      double denom = pow(params->a2, 2)
        + pow(params->a3, 2) + 2*params->a2*params->a3*c3;
      double s3 = sin(arccos);
      double A = (params->a2 + params->a3*c3), B = params->a3*s3;
      q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
      q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
      double c23_0 = cos(q2[0]+q3[0]);
      double s23_0 = sin(q2[0]+q3[0]);
      double c23_1 = cos(q2[1]+q3[1]);
      double s23_1 = sin(q2[1]+q3[1]);
      q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
      q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);

      // cleanup results
      for (int k = 0; k < 2; k++) {
        if (fabs(q2[k]) < ZERO_THRESH) {
          q2[k] = 0.0;
        } else if (q2[k] < 0.0) {
          q2[k] += 2.0*M_PI;
        }

        if (fabs(q4[k]) < ZERO_THRESH) {
          q4[k] = 0.0;
        } else if (q4[k] < 0.0) {
          q4[k] += 2.0*M_PI;
        }
        q_sols[(*num_sols)*6+0] = q1[i];
        q_sols[(*num_sols)*6+1] = q2[k];
        q_sols[(*num_sols)*6+2] = q3[k];
        q_sols[(*num_sols)*6+3] = q4[k];
        q_sols[(*num_sols)*6+4] = q5[i][j];
        q_sols[(*num_sols)*6+5] = q6;
        (*num_sols)++;
      }
    }
  }

  // at this point, we have [0, 8] solutions all qs are >= 0.0
  // use qnear_func to find the best solution
  int min_sol_idx = -1;

  qnear_func func = nullptr;
  if (qnear_mode == QNEAR_MODE_0) {
    func = qnear_err_func_0;
  } else if (qnear_mode == QNEAR_MODE_1) {
    func = qnear_err_func_1;
  } else {
    return min_sol_idx;  // unable to pick a best solution
  }

  double min_err = std::numeric_limits<double>::infinity();
  double err = std::numeric_limits<double>::infinity();
  double* min_costs = new double[kNumJointsInArm];
  double* min_vals = new double[kNumJointsInArm];
  // pick the best solution given qnear and qnear_mode
  for (int i = 0; i < (*num_sols); ++i) {
    err = func(qnear, &q_sols[kNumJointsInArm*i],
      min_costs, min_vals);
    // printf("candidate err: %.3f\n", err);
    if (err < min_err) {
      // printf("err: %.3f\n", err);
      min_err = err;
      min_sol_idx = i;
      for (int j = 0; j < kNumJointsInArm; ++j) {
        qnear_best_sol[j] = min_vals[j];
      }
    }
  }

  return min_sol_idx;
}
*/

int analytic_ur_fk(
  std::vector<DHJoint>* joints,
  std::vector<double>* qs,
  Hom* fk_hom) {
  assert(qs->size() == joints->size());
  if (qs->size() != joints->size()) {
    printf("analytic_ur_fk::mismatch between qs size and joints size!\n");
    throw stomp::Exception("analytic_ur_fk::mismatch!");
  }

  *fk_hom = Eigen::MatrixXd::Identity(4, 4);
  for (int i = 0; i < joints->size(); ++i) {
    (*joints)[i].calc_fk((*qs)[i]);

    (*fk_hom) = (*fk_hom) * (*joints)[i].a_matrix;
    // do not use noalias, you need to use temporary
  }

  return 0;
}

int analytic_ur_fk_2(
  std::vector<DHJoint>* joints,
  Eigen::MatrixXd* qs,
  Hom* fk_hom) {
  *fk_hom = Eigen::MatrixXd::Identity(4, 4);
  for (int i = 0; i < joints->size(); ++i) {
    (*joints)[i].calc_fk((*qs)(i, 0));

    (*fk_hom) = (*fk_hom) * (*joints)[i].a_matrix;
    // do not use noalias, you need to use temporary
  }

  return 0;
}

}  // namespace stomp

namespace YAML {

template <>
struct convert<stomp::DHJoint> {
  static bool decode(const YAML::Node& node,
    stomp::DHJoint& o);  // NOLINT(runtime/references)
};

// template <>
// struct convert<stomp::StompTest> {
//   static bool decode(const YAML::Node& node,
//     stomp::StompTest& s);  // NOLINT(runtime/references)
// };

}  // namespace YAML

#endif  // SRC_STOMP_INCLUDE_STOMP_UR_KIN_H_
