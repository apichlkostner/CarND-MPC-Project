#include "Config.h"
#include <fstream>
#include <iostream>
#include <string>
#include "json.hpp"

namespace mpc_project {

double Config::kDt = 0.12;
double Config::kVTarget = 120;

double Config::kFacPenaltyErrorCte = 20000.;
double Config::kFacPenaltyErrorPsi = 50000000.;
double Config::kFacPenaltyErrorVelocity = 1200;
double Config::kFacPenaltyStrengthActSteer = 20.;
double Config::kFacPenaltyStrengthActAcc = .1;
double Config::kFacPenaltyGradientSteer = 1.;
double Config::kFacPenaltyGradientAcc = .1;

size_t Config::N = 11;
size_t Config::kXStart = 0;
size_t Config::kYStart = Config::kXStart + Config::N;
size_t Config::kPsiStart = Config::kYStart + Config::N;
size_t Config::kVStart = Config::kPsiStart + Config::N;
size_t Config::kCteStart = Config::kVStart + Config::N;
size_t Config::kEpsiStart = Config::kCteStart + Config::N;
size_t Config::kDeltaStart = Config::kEpsiStart + Config::N;
size_t Config::kAStart = Config::kDeltaStart + Config::N - 1;

void Config::ReadConfig(const std::string config) {
  try {
    std::ifstream config_file(config);
    std::string config_string((std::istreambuf_iterator<char>(config_file)),
                              std::istreambuf_iterator<char>());

    nlohmann::json config_json = nlohmann::json::parse(config_string);

    Config::N = config_json["N"];
    Config::kDt = config_json["dt"];
    Config::kVTarget =
        static_cast<double>(config_json["v_target"]) * Config::kVSim2metric;
    Config::kFacPenaltyErrorCte = config_json["kFacPenaltyErrorCte"];
    Config::kFacPenaltyErrorPsi = config_json["kFacPenaltyErrorPsi"];
    Config::kFacPenaltyErrorVelocity = config_json["kFacPenaltyErrorVelocity"];
    Config::kFacPenaltyStrengthActSteer =
        config_json["kFacPenaltyStrengthActSteer"];
    Config::kFacPenaltyStrengthActAcc =
        config_json["kFacPenaltyStrengthActAcc"];
    Config::kFacPenaltyGradientSteer = config_json["kFacPenaltyGradientSteer"];
    Config::kFacPenaltyGradientAcc = config_json["kFacPenaltyGradientAcc"];
  } catch (const std::exception& e) {
    std::cout << "Exception in Config::ReadConfig. Please check your "
                 "configuration file "
              << config << std::endl;
    std::cout << "Currently some or all config parameters are initialized with the hard coded default values." << std::endl;
  }
}

}  // namespace mpc_project