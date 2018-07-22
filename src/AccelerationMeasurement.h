#ifndef ACCELERATIONMEASUREMENT_H_
#define ACCELERATIONMEASUREMENT_H_

#include <math.h>
#include <iostream>
#include <vector>
#include "Logger.h"

namespace mpc_project {

class AccelerationMeasurement {
 public:
  AccelerationMeasurement()
      : measure_log_("measurement3.csv"), throttle_value_(1.)  {}

  double GetNewThrottle(double v, double px, double py, double dt) {
    static double throttle_test = 1.0;
    {
      static int cnt;

      constexpr int BRAKE_AFTER = 70;
      if (++cnt > BRAKE_AFTER) {
        throttle_value_ = -0.2;
        cnt = 0;
      }
      const double a = (v - v_last_) / dt;
      const double s = sqrt((px - x_last_) * (px - x_last_) +
                            (py - y_last_) * (py - y_last_));
      std::cout << " v = " << v << " s = " << s << " s_expected = " << v * dt
                << " a = " << a << " throttle " << throttle_test << std::endl;

      std::vector<double> logdata{throttle_test, v, a};
      measure_log_.log("", logdata);
      v_last_ = v;
      x_last_ = px;
      y_last_ = py;
    }
    return throttle_value_;
  }

 private:
  Logger measure_log_;
  double throttle_value_;
  double v_last_;
  double x_last_;
  double y_last_;
};
}  // namespace mpc_project
#endif