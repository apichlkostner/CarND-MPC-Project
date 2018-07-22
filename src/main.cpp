#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Logger.h"
#include "MPC.h"
#include "Vehicle.h"
#include "constants.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using namespace std;
using namespace mpc_project;

#define ACC_TEST 0

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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    static auto last_call_time = std::chrono::system_clock::now();
    static auto first_call_time = std::chrono::system_clock::now();
    static Vehicle vehicle;

#if ACC_TEST
    static double v_last, x_last, y_last;
    static Logger measure_log("measurement3.csv");
#endif

    auto current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> call_dur = current_time - last_call_time;
    std::chrono::duration<double> run_dur = current_time - first_call_time;
#if !ACC_TEST
    cout << "Running for " << run_dur.count()
         << " Call time = " << call_dur.count() << endl;
#endif
    last_call_time = std::chrono::system_clock::now();

    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        static double time_average = 0.;
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
          v *= constants::kVSim2metric;

#if ACC_TEST
          static double throttle_test = 1.0;
          {
            static int cnt;

            if (++cnt > 70) {
              // std::string msg = "42[\"reset\",{}]";
              // ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

              throttle_test = -0.2;
              cnt = 0;
            }
            const double a = (v - v_last) / call_dur.count();
            const double s = sqrt((px - x_last) * (px - x_last) +
                                  (py - y_last) * (py - y_last));
            cout << " v = " << v << " s = " << s
                 << " s_expected = " << v * call_dur.count() << " a = " << a
                 << " throttle " << throttle_test << endl;

            vector<double> logdata{throttle_test, v, a};
            measure_log.log("", logdata);
            v_last = v;
            x_last = px;
            y_last = py;
          }
#endif

          // cross track error
          assert(ptsx.size() == ptsy.size());

          Eigen::VectorXd ptsx_loc(ptsx.size());
          Eigen::VectorXd ptsy_loc(ptsy.size());

          for (size_t i = 0; i < ptsx.size(); i++) {
            const double dx = ptsx[i] - px;
            const double dy = ptsy[i] - py;

            ptsx_loc[i] = dx * cos(-psi) - dy * sin(-psi);
            ptsy_loc[i] = dx * sin(-psi) + dy * cos(-psi);
          }

          auto coeffs = polyfit(ptsx_loc, ptsy_loc, 3);
          double epsi = atan(coeffs[1]);
#if 1
          double cte = polyeval(coeffs, 0);
#else
          double cte = cos(epsi) * polyeval(coeffs, 0);
#endif

          Eigen::VectorXd state(6);
          // in car refernce system pos_x = pos_y = psi = 0
          state << 0, 0, 0, v, cte, epsi;

          auto start_time = std::chrono::system_clock::now();

          auto vars = mpc.Solve(state, coeffs);

          auto end_time = std::chrono::system_clock::now();
          std::chrono::duration<double> dur = end_time - start_time;
          double calc_time = dur.count();
          constexpr double fak_mov_average = 0.99;

          if (time_average == 0.)
            time_average = calc_time;
          else
            time_average = fak_mov_average * time_average +
                           (1. - fak_mov_average) * calc_time;

#if !ACC_TEST
          std::cout << "Average time = " << time_average
                    << " Calc time = " << calc_time << std::endl;
#endif

          // accumulated_time += calc_time;

          double steer_value = vars[0];
          double acceleration = vars[1];

          double throttle_value = vehicle.CalcThrottle(acceleration, v);

          cout << "a = " << acceleration << " v = " << v << " throttle = " << throttle_value
               << endl;

#if ACC_TEST
          throttle_value = throttle_test;
#endif

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          // steering value back. Otherwise the values will be in between
          // [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          //(mat.data(), mat.data() + mat.rows() * mat.cols());

          vector<double> mpc_y_vals;

          for (size_t i = 0; i < constants::N; i++) {
            mpc_x_vals.push_back(vars[2 + 2 * i]);
            mpc_y_vals.push_back(vars[3 + 2 * i]);
          }

          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

//.. add (x,y) points to list here, points are in reference to the
// vehicle's coordinate system
// the points in the simulator are connected by a Yellow line
#if 1
          for (int i = 0; i < 35; i++) {
            double dx = i * 2;
            next_x_vals.push_back(dx);
            next_y_vals.push_back(polyeval(coeffs, dx));
          }
#else
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
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
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
