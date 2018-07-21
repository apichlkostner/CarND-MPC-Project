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
};
}  // namespace mpc_project

#endif