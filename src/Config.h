#ifndef CONFIG_H_
#define CONFIG_H_

#include <string>
#include <vector>

// Enable acceleration test to measure the dependencies
// between throttle, velocity and acceleration of the vehicle
#define ACC_TEST 0
// 1 -> draw polynomials as reference
// 0 -> draw waypoints as reference)
#define DRAW_REFERENCE_POLYNOMIAL 0

// Enable time measurement
#define TIME_MEASUREMENT 1

namespace mpc_project {
class Config {
 public:
  static void ReadConfig(const std::string config);

  static size_t N;
  static double kDt;
  static double kVTarget;

  // penalty factor for cost function
  static double kFacPenaltyErrorCte;
  static double kFacPenaltyErrorPsi;
  static double kFacPenaltyErrorVelocity;
  static double kFacPenaltyStrengthActSteer;
  static double kFacPenaltyStrengthActAcc;
  static double kFacPenaltyGradientSteer;
  static double kFacPenaltyGradientAcc;

  static constexpr double kLf = 2.67;
  static constexpr double kMph2mps = 0.447;
  static constexpr double kVSim2metric = kMph2mps;
  static constexpr double kBrakeCoef = 20.;

  static size_t kXStart;
  static size_t kYStart;
  static size_t kPsiStart;
  static size_t kVStart;
  static size_t kCteStart;
  static size_t kEpsiStart;
  static size_t kDeltaStart;
  static size_t kAStart;
};
}  // namespace mpc_project

#endif