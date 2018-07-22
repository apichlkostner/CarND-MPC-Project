#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <vector>

namespace mpc_project {
class constants {
 public:
  static constexpr size_t N = 11;
  static constexpr double facPenaltyAct = 5.;
  static constexpr double facPenaltyActChange = 1.;
  static constexpr double Lf = 2.67;
  static constexpr double kMph2mps = 0.44;
  static constexpr double kVSim2metric = 1.;
};
}  // namespace mpc_project

#endif