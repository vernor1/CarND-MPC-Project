#ifndef MODELPREDICTIVECONTROLLER_H
#define MODELPREDICTIVECONTROLLER_H

#include <functional>
#include <vector>
#include "MpcSolver.h"

class ModelPredictiveController {
public:
  typedef std::function<void(
    double steering,
    double throttle,
    const std::vector<double>& predicted_x,
    const std::vector<double>& predicted_y,
    const std::vector<double>& reference_x,
    const std::vector<double>& reference_y)> ControlFunction;

  // Contructor.
  ModelPredictiveController();

  // Destructor.
  virtual ~ModelPredictiveController() { }

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
