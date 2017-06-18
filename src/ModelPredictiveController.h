#ifndef MODELPREDICTIVECONTROLLER_H
#define MODELPREDICTIVECONTROLLER_H

#include <functional>
#include <vector>
#include "MpcSolver.h"

// Implements MPC by processing the telemetry data and providing the steering
// angle and trottle value. Aggregates an instance of MPC Solver.
class ModelPredictiveController {
public:
  // The functor is called upon completion of the controller update.
  // @param steering  Steering angle in [-1..1].
  // @param throttle  Throttle value in [-1..1].
  // @param predicted_x  X-coordinates of the predicted waypoints.
  // @param predicted_y  Y-coordinates of the predicted waypoints.
  // @param reference_x  X-coordinates of the reference waypoints.
  // @param reference_y  Y-coordinates of the reference waypoints.
  typedef std::function<void(
    double steering,
    double throttle,
    const std::vector<double>& predicted_x,
    const std::vector<double>& predicted_y,
    const std::vector<double>& reference_x,
    const std::vector<double>& reference_y)> ControlFunction;

  // Contructor.
  // @param lf        Distance between the front of the vehicle and its center
  //                  of gravity.
  // @param dt        Time difference between predicted waypoints.
  // @param n_points  Number of points in the predicted path.
  // @param s_points  Number of nearest predicted waypoints to be skipped
  //                  because of latency.
  // @param v         Desired velocity in [mph].
  ModelPredictiveController(double lf,
                            double dt,
                            size_t n_points,
                            size_t s_points,
                            double v);

  // Destructor.
  virtual ~ModelPredictiveController() { }

  // Updates this MPC with the new telemetry data.
  // @param waypoints_x       X-coordinates of the reference waypoints.
  // @param waypoints_y       Y-coordinates of the reference waypoints.
  // @param x                 X-coordinate of the vehicle.
  // @param y                 Y-coordinate of the vehicle.
  // @param yaw               Yaw of the vehicle in [rad].
  // @param speed             Current speed in [mph].
  // @param control_function  Functor to control the simulator.
  void Update(const std::vector<double>& waypoints_x,
              const std::vector<double>& waypoints_y,
              double x,
              double y,
              double yaw,
              double speed,
              ControlFunction control_function);
private:
  MpcSolver solver_;
};

#endif // MODELPREDICTIVECONTROLLER_H
