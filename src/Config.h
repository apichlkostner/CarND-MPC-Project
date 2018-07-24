#ifndef CONFIG_H_
#define CONFIG_H_

// Enable acceleration test to measure the dependencies
// between throttle, velocity and acceleration of the vehicle
#define ACC_TEST 1
// 1 -> draw polynomials as reference
// 0 -> draw waypoints as reference)
#define DRAW_REFERENCE_POLYNOMIAL 0

// Enable time measurement
#define TIME_MEASUREMENT 1

namespace mpc_project {
class Config {
 public:
  static constexpr size_t N = 11;
  static constexpr double kDt = 0.12;
  static constexpr double kVTarget = 120;

  // penalty factor for cost function
  static constexpr double kFacPenaltyErrorCte = 3000.;
  static constexpr double kFacPenaltyErrorPsi = 30000000.;
  static constexpr double kFacPenaltyErrorVelocity = 150;
  static constexpr double kFacPenaltyStrengthActSteer = 2000.;
  static constexpr double kFacPenaltyStrengthActAcc = 3.;
  static constexpr double kFacPenaltyGradientSteer = 100.;
  static constexpr double kFacPenaltyGradientAcc = 5.;

  static constexpr double kLf = 2.67;
  static constexpr double kMph2mps = 0.447;
  static constexpr double kVSim2metric = kMph2mps;
  static constexpr double kBrakeCoef = 20.;
};
}  // namespace mpc_project

#endif