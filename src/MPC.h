#ifndef MPC_H
#define MPC_H

#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Vehicle.h"
#include "json.hpp"

namespace mpc_project {
class MPC {
 public:
  MPC() : last_state_(6), vehicle() {
    last_state_ << -10000, 0, 0, 0, 0, 0;
  }

  MPC(const std::string config);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

 private:
  Eigen::VectorXd last_state_;
  Vehicle vehicle;
};
}  // namespace mpc_project
#endif /* MPC_H */
