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

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// TCP port accepting incoming connections from simulator
enum { kTcpPort = 4567 };

enum { kDefaultN = 12 };

enum { kDefaultS = 2 };

const auto kDefaultLf = 2.67;

const auto kDefaultDt = 0.15;

const auto kDefaultV = 80;

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
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

// Checks arguments of the program and exits, if the check fails.
// @param[in] argc  Number of arguments
// @param[in] argv  Array of arguments
// @return          A smart pointer to the PID controller object
std::shared_ptr<ModelPredictiveController> CreateMpc(int argc, char* argv[]) {
  std::stringstream oss;
    oss << "Usage instructions: " << argv[0] << " [Lf dt N S V]" << std::endl
        << "  Lf  Distance between the front of the vehicle and its center of"
        << " gravity (" << kDefaultLf << " by default)" << std::endl
        << "  dt  Time difference between predicted waypoints (" << kDefaultDt
        << " by default)" << std::endl
        << "  N   Number of predicted waypoints (" << kDefaultN
        << " by default)" << std::endl
        << "  S   Number of nearest waypoints to skip on control due to"
        << " the latency (" << kDefaultS << " by default)" << std::endl
        << "  V   Desired speed in [mph] (" << kDefaultV << " by default)"
        << std::endl;

  std::shared_ptr<ModelPredictiveController> mpc;
  try {
    switch (argc) {
      case 1:
        mpc.reset(new ModelPredictiveController(kDefaultLf,
                                                kDefaultDt,
                                                kDefaultN,
                                                kDefaultS,
                                                kDefaultV));
        break;
      case 6: {
        auto lf = std::stod(argv[1]);
        auto dt = std::stod(argv[2]);
        auto n = std::stoul(argv[3]);
        auto s = std::stoul(argv[4]);
        auto v = std::stod(argv[5]);
        mpc.reset(new ModelPredictiveController(lf, dt, n, s, v));
        break;
      }
      default:
        std::cerr << "Error: invalid number of arguments" << std::endl << oss.str();
        std::exit(EXIT_FAILURE);
    }
  }
  catch (const std::exception& e) {
    std::cerr << "Error: invalid data format: " << e.what() << std::endl
              << oss.str();
    std::exit(EXIT_FAILURE);
  }

  return mpc;
}

} // namespace

int main(int argc, char* argv[]) {
  uWS::Hub hub;
  auto mpc = CreateMpc(argc, argv);
  hub.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws,
                       char* data,
                       size_t length,
                       uWS::OpCode op_code) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 indicates a websocket message
    // The 2 indicates a websocket event
    auto sdata = std::string(data).substr(0, length);
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
          mpc->Update(waypoints_x, waypoints_y,
                      x, y, yaw, speed,
                      std::bind(ControlSimulator, ws, _1, _2, _3, _4, _5, _6));
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  hub.onConnection([&hub](uWS::WebSocket<uWS::SERVER> ws,
                          uWS::HttpRequest req) {
    std::cout << "WebSocket connected!!!" << std::endl;
  });

  hub.onDisconnection([&hub](uWS::WebSocket<uWS::SERVER> ws,
                             int code,
                             char* message,
                             size_t length) {
    ws.close();
    std::cout << "WebSocket disconnected" << std::endl;
  });

  if (hub.listen(kTcpPort)) {
    std::cout << "Listening on port " << kTcpPort << std::endl;
  } else {
    std::cerr << "Failed to listen on port " << kTcpPort << std::endl;
    return -1;
  }

  hub.run();
}
