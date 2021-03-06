#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Config.h"
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using namespace std;

namespace mpc_project {
// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t& kXStart = Config::kXStart;
size_t& kYStart = Config::kYStart;
size_t& kPsiStart = Config::kPsiStart;
size_t& kVStart = Config::kVStart;
size_t& kCteStart = Config::kCteStart;
size_t& kEpsiStart =Config::kEpsiStart;
size_t& kDeltaStart = Config::kDeltaStart;
size_t& kAStart = Config::kAStart;

class FG_eval {
 public:
  // Coefficients of the fitted polynomial.
  FG_eval(Eigen::VectorXd coeffs, double v_target)
      : coeffs_(coeffs), v_target_(v_target){};

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    constexpr size_t kCostPos = 0;
    fg[kCostPos] = 0;

    // Reference State Cost
    for (size_t t = 0; t < Config::N; t++) {
      fg[kCostPos] +=
          Config::kFacPenaltyErrorCte * CppAD::pow(vars[kCteStart + t], 2);
      fg[kCostPos] +=
          Config::kFacPenaltyErrorPsi * CppAD::pow(vars[kEpsiStart + t], 2);
      fg[kCostPos] += Config::kFacPenaltyErrorVelocity *
                      CppAD::pow(vars[kVStart + t] - v_target_, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < Config::N - 1; t++) {
      fg[kCostPos] += Config::kFacPenaltyStrengthActSteer *
                      CppAD::pow(vars[kDeltaStart + t], 2);
      fg[kCostPos] +=
          Config::kFacPenaltyStrengthActAcc * CppAD::pow(vars[kAStart + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < Config::N - 2; t++) {
      fg[kCostPos] +=
          Config::kFacPenaltyGradientSteer *
          CppAD::pow(vars[kDeltaStart + t + 1] - vars[kDeltaStart + t], 2);
      fg[kCostPos] += Config::kFacPenaltyGradientAcc *
                      CppAD::pow(vars[kAStart + t + 1] - vars[kAStart + t], 2);
    }

    //
    // Setup Constraints
    //

    // Initial constraints
    constexpr size_t kConstrStart = 1;

    fg[kConstrStart + kXStart] = vars[kXStart];
    fg[kConstrStart + kYStart] = vars[kYStart];
    fg[kConstrStart + kPsiStart] = vars[kPsiStart];
    fg[kConstrStart + kVStart] = vars[kVStart];
    fg[kConstrStart + kCteStart] = vars[kCteStart];
    fg[kConstrStart + kEpsiStart] = vars[kEpsiStart];

    // The rest of the constraints
    for (size_t t = kConstrStart; t < Config::N; t++) {
      AD<double> x1 = vars[kXStart + t];
      AD<double> y1 = vars[kYStart + t];
      AD<double> y0 = vars[kYStart + t - 1];
      AD<double> x0 = vars[kXStart + t - 1];
      AD<double> psi0 = vars[kPsiStart + t - 1];
      AD<double> psi1 = vars[kPsiStart + t];
      AD<double> v0 = vars[kVStart + t - 1];
      AD<double> v1 = vars[kVStart + t];
      AD<double> delta0 = vars[kDeltaStart + t - 1];
      AD<double> f0 = coeffs_[0] + coeffs_[1] * x0 + coeffs_[2] * x0 * x0 +
                      coeffs_[3] * x0 * x0 * x0;
      AD<double> psi_des0 = CppAD::atan(3 * coeffs_[3] * x0 * x0 +
                                        2 * coeffs_[2] * x0 + coeffs_[1]);
      AD<double> a0 = vars[kAStart + t - 1];
      AD<double> cte0 = vars[kCteStart + t - 1];
      AD<double> epsi0 = vars[kEpsiStart + t - 1];
      AD<double> cte1 = vars[kCteStart + t];
      AD<double> epsi1 = vars[kEpsiStart + t];

      fg[kConstrStart + kXStart + t] =
          x1 - (x0 + v0 * CppAD::cos(psi0) * Config::kDt);
      fg[kConstrStart + kYStart + t] =
          y1 - (y0 + v0 * CppAD::sin(psi0) * Config::kDt);
      fg[kConstrStart + kPsiStart + t] =
          psi1 - (psi0 + v0 / Config::kLf * delta0 * Config::kDt);
      fg[kConstrStart + kVStart + t] = v1 - (v0 + a0 * Config::kDt);
      fg[kConstrStart + kCteStart + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * Config::kDt));
      fg[kConstrStart + kEpsiStart + t] =
          epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Config::kLf * Config::kDt);
    }
  }

 private:
  Eigen::VectorXd coeffs_;
  double v_target_;
};



//
// MPC class definition implementation.
//
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // number of independent variables
  // N timesteps == N - 1 actuations
  const size_t n_vars = Config::N * 6 + (Config::N - 1) * 2;
  // Number of constraints
  const size_t n_constraints = Config::N * 6;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[kXStart] = x;
  vars[kYStart] = y;
  vars[kPsiStart] = psi;
  vars[kVStart] = v;
  vars[kCteStart] = cte;
  vars[kEpsiStart] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < kDeltaStart; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (size_t i = kDeltaStart; i < kAStart; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  if (last_state_[0] != -10000) {
    vars_lowerbound[kDeltaStart] = last_state_[3];
    vars_upperbound[kDeltaStart] = last_state_[3];
  }

  // Acceleration/decceleration upper and lower limits.
  for (size_t i = kAStart; i < n_vars; i++) {
    vars_lowerbound[i] = vehicle.MinAcceleration(v);
    vars_upperbound[i] = vehicle.MaxAcceleration(v);
  }

  if (last_state_[0] != -10000) {
    vars_lowerbound[kAStart] = last_state_[5];
    vars_upperbound[kAStart] = last_state_[5];
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[kXStart] = x;
  constraints_lowerbound[kYStart] = y;
  constraints_lowerbound[kPsiStart] = psi;
  constraints_lowerbound[kVStart] = v;
  constraints_lowerbound[kCteStart] = cte;
  constraints_lowerbound[kEpsiStart] = epsi;

  constraints_upperbound[kXStart] = x;
  constraints_upperbound[kYStart] = y;
  constraints_upperbound[kPsiStart] = psi;
  constraints_upperbound[kVStart] = v;
  constraints_upperbound[kCteStart] = cte;
  constraints_upperbound[kEpsiStart] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, Config::kVTarget);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.15\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  vector<double> retsol;
  retsol.push_back(solution.x[kDeltaStart + 2]);
  retsol.push_back(solution.x[kAStart + 2]);

  for (size_t i = 0; i < Config::N; i++) {
    retsol.push_back(solution.x[kXStart + i]);
    retsol.push_back(solution.x[kYStart + i]);
  }

  Eigen::VectorXd lastState(6);
  lastState << solution.x[kXStart + 2], solution.x[kYStart + 2],
      solution.x[kVStart + 2], solution.x[kDeltaStart + 2],
      solution.x[kEpsiStart + 2], solution.x[kAStart + 2];
  last_state_ = std::move(lastState);

  return retsol;
}
}  // namespace mpc_project