#ifndef TIMEUTILS_H_
#define TIMEUTILS_H_

#include <chrono>
#include <iostream>
#include "Config.h"

namespace mpc_project {
class TimeMeasurement {
 public:
  TimeMeasurement() {
#if TIME_MEASUREMENT
    last_call_time_ = std::chrono::system_clock::now();
    first_call_time_ = std::chrono::system_clock::now();
#endif
  }

  void BeginGlobalMeasurement() {
#if TIME_MEASUREMENT
    auto current_time = std::chrono::system_clock::now();

    std::chrono::duration<double> call_dur = current_time - last_call_time_;
    std::chrono::duration<double> run_dur = current_time - first_call_time_;

    dt_glob_ = run_dur.count();

    last_call_time_ = std::chrono::system_clock::now();

    std::cout << "Running for " << dt_glob_ << " Call time = " << dt_glob_
              << std::endl;
#endif
  }

  void StartInnerMeasurement() {
#if TIME_MEASUREMENT
    start_time_ = std::chrono::system_clock::now();
#endif
  }

  void EndInnerMeasurement() {
#if TIME_MEASUREMENT
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> dur = end_time - start_time_;
    double calc_time = dur.count();
    constexpr double fak_mov_average = 0.99;

    if (time_average_ == 0.)
      time_average_ = calc_time;
    else
      time_average_ =
          fak_mov_average * time_average_ + (1. - fak_mov_average) * calc_time;

    std::cout << "Average time = " << time_average_
              << " Calc time = " << calc_time << std::endl;
#endif
  }

  double DtGlob() {
#if TIME_MEASUREMENT
    return dt_glob_;
#else
    assert(0. != 0.);
    return 0.;
#endif
  }

 private:
#if TIME_MEASUREMENT
  std::chrono::time_point<std::chrono::system_clock> last_call_time_;
  std::chrono::time_point<std::chrono::system_clock> first_call_time_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;

  double time_average_;
  double dt_glob_;
#endif
};
}  // namespace mpc_project
#endif
