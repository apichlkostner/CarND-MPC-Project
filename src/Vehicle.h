#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "Eigen-3.3/Eigen/Core"
#include "constants.h"

namespace mpc_project {

class Vehicle {
 public:
  Vehicle() : acc_coef_(3), throttle_coef_(3) {
    acc_coef_ << 1.20e+01, 9.7e-02, 8.0e-05;
    throttle_coef_ << 8.05e-02, 8.04e-03, 6.64e-06;
  }

  virtual ~Vehicle() {}

  double CalcAcceleration(double throttle, double v) {
    v /= constants::kVSim2metric;

    double a;
    if (throttle > 0.) {
      a = acc_coef_[0] * throttle + acc_coef_[1] * v + acc_coef_[2] * v * v;
    } else {
      // Good model for braking is missing
      a = throttle * constants::kBrakeCoef;
    }

    return a;
  }

  double MaxAcceleration(double v) {
    // maximum acceleration with maximum trottle
    return CalcAcceleration(1., v);
  }

  double MinAcceleration(double v) {
    // brake force seems to be constant for higher velocities
    // since the vehicle is always relatively fast this is precise enough
    return -constants::kBrakeCoef;
  }

  double CalcThrottle(double a, double v) {
    v /= constants::kVSim2metric;

    double throttle;

    if (a > 0.) {
      throttle = throttle_coef_[0] * a + throttle_coef_[1] * v +
                 throttle_coef_[2] * v * v;
    } else {
      // Good model for braking is missing
      throttle = a / constants::kBrakeCoef;
    }

    if (throttle > 1.)
      throttle = 1.;
    else if (throttle < -1.)
      throttle = -1.;

    return throttle;
  }

 private:
  Eigen::VectorXd acc_coef_;
  Eigen::VectorXd throttle_coef_;
};
}  // namespace mpc_project
#endif