#ifndef CONFIG_H_
#define CONFIG_H_

// Enable acceleration test to measure the dependencies
// between throttle, velocity and acceleration of the vehicle
#define ACC_TEST 0
// 1 -> draw polynomials as reference
// 0 -> draw waypoints as reference)
#define DRAW_REFERENCE_POLYNOMIAL 1

// Enable time measurement
#define TIME_MEASUREMENT 1

namespace mpc_project {
class Config {
 public:
  static constexpr size_t N = 11;
  static constexpr double kDt = 0.11;
  static constexpr double kVTarget = 90;

  // penalty factor for cost function
  static constexpr double kFacPenaltyErrorCte = 6000.;
  static constexpr double kFacPenaltyErrorPsi = 30000.;
  static constexpr double kFacPenaltyErrorVelocity = 0.6;
  static constexpr double kFacPenaltyStrengthActSteer = 3.;
  static constexpr double kFacPenaltyStrengthActAcc = 3.;
  static constexpr double kFacPenaltyGradientSteer = 200.;
  static constexpr double kFacPenaltyGradientAcc = 10.;

  static constexpr double kLf = 2.67;
  static constexpr double kMph2mps = 0.44;
  static constexpr double kVSim2metric = 1.;
  static constexpr double kBrakeCoef = 20.;
};
}  // namespace mpc_project

#endif