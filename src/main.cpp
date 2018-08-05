#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "AccelerationMeasurement.h"
#include "Config.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Logger.h"
#include "MPC.h"
#include "TimeUtils.h"
#include "Vehicle.h"
#include "json.hpp"
#include <thread>

// for convenience
using json = nlohmann::json;
using namespace std;
using namespace mpc_project;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

void ReadConfigThread() {
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "Update Config" << std::endl;
    Config::ReadConfig("../src/Config.json");
  } while(true);
}

int main() {
  // Initialization of configuration
  Config::ReadConfig("../src/Config.json");

  // vehicle dependent calculations
  Vehicle vehicle;
   
  // Model Predicitve Controller
  MPC mpc;

  // Server to connect with simulator
  uWS::Hub h;

  std::thread thread1(ReadConfigThread);

  // Callback for communication with simulator
  h.onMessage([&mpc, &vehicle](uWS::WebSocket<uWS::SERVER> ws, char *data,
                               size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    // time measurement
    static TimeMeasurement time_meas;
    time_meas.BeginGlobalMeasurement();

    string sdata = string(data).substr(0, length);

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // change unit if necessary
          v *= Config::kVSim2metric;

          // cross track error
          assert(ptsx.size() == ptsy.size());

          Eigen::VectorXd ptsx_loc(ptsx.size());
          Eigen::VectorXd ptsy_loc(ptsy.size());

          // transform waypoints from global to cars reference system
          for (size_t i = 0; i < ptsx.size(); i++) {
            const double dx = ptsx[i] - px;
            const double dy = ptsy[i] - py;

            ptsx_loc[i] = dx * cos(-psi) - dy * sin(-psi);
            ptsy_loc[i] = dx * sin(-psi) + dy * cos(-psi);
          }

          // fit polynomial and calculate first error values
          auto coeffs = polyfit(ptsx_loc, ptsy_loc, 3);
          double e_psi = atan(coeffs[1]);
          double cte = polyeval(coeffs, 0);

          Eigen::VectorXd state(6);
          // in car refernce system pos_x = pos_y = psi = 0
          //       x  y psi v  cte  error_psi
          state << 0, 0, 0, v, cte, e_psi;

          time_meas.StartInnerMeasurement();

          ///
          /// solve mpc equations
          ///
          auto vars = mpc.Solve(state, coeffs);

          time_meas.EndInnerMeasurement();

          // first two values are steering angle and acceleration
          double steer_value = vars[0];  // degree
          double acceleration = vars[1];

          // throttle value from given acceration
          double throttle_value = vehicle.CalcThrottle(acceleration, v);

          cout << "a = " << acceleration << " v = " << v
               << " throttle = " << throttle_value << endl;

#if ACC_TEST
          // override throttle value to test different values to build a model
          static AccelerationMeasurement acc_meas;

          throttle_value =
              acc_meas.GetNewThrottle(v, px, py, time_meas.DtGlob());
#endif

          json msgJson;

          msgJson["steering_angle"] = -steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

#if 1
          static double last_pred_pos_x;
          static double last_pred_pos_y;
          static double last_pos_x;
          static double last_pos_y;

          // cout << "predicted (" << last_pred_pos_x << ", " << last_pred_pos_y
          //      << ") real (" << px << ", " << py << ")" << std::endl;

          double pred_dist = sqrt(last_pred_pos_x * last_pred_pos_x +
                                  last_pred_pos_y * last_pred_pos_y);
          double dex = px - last_pos_x;
          double dey = py - last_pos_y;
          double real_dist = sqrt(dex * dex + dey * dey);
          cout << "predicted " << pred_dist << " real " << real_dist << endl;

          last_pred_pos_x = vars[4];
          last_pred_pos_y = vars[5];
          last_pos_x = px;
          last_pos_y = py;
#endif
          for (size_t i = 0; i < Config::N; i++) {
            mpc_x_vals.push_back(vars[2 + 2 * i]);
            mpc_y_vals.push_back(vars[3 + 2 * i]);
          }

          // show predicted waypoints as green line in simulator
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // display waypoints/reference line as yellow line in simulator
          vector<double> next_x_vals;
          vector<double> next_y_vals;

#if DRAW_REFERENCE_POLYNOMIAL
          // yellow reference trajectory -> polynomial
          for (int i = 0; i < 35; i++) {
            double dx = i * 2;
            next_x_vals.push_back(dx);
            next_y_vals.push_back(polyeval(coeffs, dx));
          }
#else
          // yellow reference trajectory -> original waypoints
          for (size_t i = 0; i < ptsx.size(); i++) {
            next_x_vals.push_back(ptsx_loc[i]);
            next_y_vals.push_back(ptsy_loc[i]);
          }
#endif
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(100));

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
