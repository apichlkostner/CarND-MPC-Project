#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Vehicle.h"
namespace mpc_project {
class MPC {
 private:
  Eigen::VectorXd last_state_;
  Vehicle vehicle;

 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};
}  // namespace mpc_project
#endif /* MPC_H */
