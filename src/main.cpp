#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "ModelPredictiveController.h"

using namespace std::placeholders;

// Local Constants
// -----------------------------------------------------------------------------

// TCP port accepting incoming connections from simulator
enum { kTcpPort = 4567 };

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Checks if the SocketIO event has JSON data.
// @param[in] s  Raw event string
// @return       If there is data the JSON object in string format will be
//               returned, else the empty string will be returned.
std::string GetJsonData(const std::string& s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  return found_null == std::string::npos
    && b1 != std::string::npos
    && b2 != std::string::npos ? s.substr(b1, b2 - b1 + 1) : std::string();
}

void ControlSimulator(uWS::WebSocket<uWS::SERVER>& ws,
                      double steering,
                      double throttle,
                      const std::vector<double>& predicted_x,
                      const std::vector<double>& predicted_y,
                      const std::vector<double>& reference_x,
                      const std::vector<double>& reference_y) {
  nlohmann::json json_msg;
  json_msg["steering_angle"] = steering;
  json_msg["throttle"] = throttle;
  json_msg["mpc_x"] = predicted_x;
  json_msg["mpc_y"] = predicted_y;
  json_msg["next_x"] = reference_x;
  json_msg["next_y"] = reference_y;
  auto msg = "42[\"steer\"," + json_msg.dump() + "]";
  // Latency
  // The purpose is to mimic real driving conditions where
  // the car does actuate the commands instantly.
  //
  // Feel free to play around with this value but should be to drive
  // around the track with 100ms latency.
  //
  // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
  // SUBMITTING.
//  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

// for convenience
//using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
/*
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
*/
/*
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
*/
int main() {
  uWS::Hub h;

  // MPC is initialized here!
//  MPC mpc;
  ModelPredictiveController mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws,
                     char* data,
                     size_t length,
                     uWS::OpCode op_code) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 indicates a websocket message
    // The 2 indicates a websocket event
    auto sdata = std::string(data).substr(0, length);
//    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = GetJsonData(sdata);
      if (!s.empty()) {
        auto j = nlohmann::json::parse(s);
        auto event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::vector<double> waypoints_x = j[1]["ptsx"];
          std::vector<double> waypoints_y = j[1]["ptsy"];
          double x = j[1]["x"];
          double y = j[1]["y"];
          double yaw = j[1]["psi"];
          double speed = j[1]["speed"];
          mpc.Update(waypoints_x, waypoints_y,
                     x, y, yaw, speed,
                     std::bind(ControlSimulator, ws, _1, _2, _3, _4, _5, _6));
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
/*
          double steer_value;
          double throttle_value;

          nlohmann::json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          std::vector<double> mpc_x_vals;
          std::vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
*/
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
