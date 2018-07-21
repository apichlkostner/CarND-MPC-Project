#ifndef LOGGER_H_
#define LOGGER_H_

#include <fstream>
#include <ios>
#include <iostream>
#include <vector>

namespace mpc_project {
class Logger {
 public:
  Logger(std::string filename) {
    file_.open(filename, std::ios_base::out | std::ios_base::app);
  }

  Logger() : Logger("parameter.csv") {}

  virtual ~Logger() { file_.close(); }

  void log(std::string log, const std::vector<double>& p,
           const std::vector<double>& dp, double error, double best_err) {
    file_ << log;
    for (const auto v : p) file_ << ", " << v;
    for (const auto v : dp) file_ << ", " << v;
    file_ << ", " << error << ", " << best_err;
    file_ << std::endl;
  }

  void log(std::string log, const std::vector<double>& values) {
    file_ << log;
    for (const auto v : values) file_ << ", " << v;
    file_ << std::endl;
  }

 private:
  std::ofstream file_;
};
}  // namespace mpc_project
#endif