#include "ModelPredictiveController.h"
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// Order of the polinomial to fit the reference waypoints.
enum { kPolynomialOrder = 3 };

// Meters in mile per international agreement of 1959,
const auto kMetersInMile = 1609.344;

// Coefficient of conversion miles-per-hour to meters-per-second.
const auto kMphToMps = kMetersInMile / (60. * 60.);

// Max steering angle of the vehicle (25 degrees) in [rad].
const auto kMaxSteering = 25. / 180. * M_PI;

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Evaluates a polynomial.
// @param[in] coeffs  Polynomial coefficients.
// @param[in] x       Argument to avaluate the polynomial for.
// @return            Value of the function.
double EvaluatePolynomial(const std::vector<double>& coeffs, double x) {
  double result = 0;
  for (auto i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * std::pow(x, i);
  }
  return result;
}

// Evaluates the 1st derivative of a polynomial.
// @param[in] coeffs  Polynomial coefficients.
// @param[in] x       Argument to avaluate the derivative for.
// @return            Value of the function derivative.
double EvaluateDerivative(const std::vector<double>& coeffs, double x) {
  auto n_coeffs = coeffs.size();
  assert(n_coeffs > 0);
  double result = 0;
  for (auto i = 1; i < n_coeffs; ++i) {
    result += i * coeffs[i] * std::pow(x, i - 1);
  }
  return result;
}

// Converts absolute map coordinates to the relative coordinate system of the
// vehicle.
// @param[in] abs_x  Absolute X-coordinates.
// @param[in] abs_y  Absolute Y-coordinates.
// @param[in] x      X-coordinate of the vehicle.
// @param[in] y      Y-coordinate of the vehicle.
// @param[in] yaw    Yaw of the vehicle.
// @param[in] rel_x  Relative X-coordinates.
// @param[in] rel_y  Relative Y-coordinates.
void AbsoluteToRelativeCoords(const Eigen::VectorXd& abs_x,
                              const Eigen::VectorXd& abs_y,
                              double x,
                              double y,
                              double yaw,
                              Eigen::VectorXd& rel_x,
                              Eigen::VectorXd& rel_y) {
  assert(abs_x.size() == abs_y.size());
  auto cos_a = std::cos(yaw);
  auto sin_a = std::sin(yaw);
  auto diff_x = abs_x.array() - x;
  auto diff_y = abs_y.array() - y;
  rel_x = diff_x * cos_a + diff_y * sin_a;
  rel_y = diff_y * cos_a - diff_x * sin_a;
}

// Fits a polynomial to coordinates.
// @param[in] x      Eigen vector of X-coordinates.
// @param[in] y      Eigen vector of Y-coordinates.
// @param[in] order  Order of the polynomial.
// @return           Polynomial coefficients.
std::vector<double> FitPolynomial(Eigen::VectorXd x,
                                  Eigen::VectorXd y,
                                  unsigned int order) {
  assert(x.size() == y.size());
  assert(order > 0 && order <= x.size() - 1);
  Eigen::MatrixXd a(x.size(), order + 1);
  for (auto i = 0; i < x.size(); ++i) {
    a(i, 0) = 1;
  }
  for (auto j = 0; j < x.size(); ++j) {
    for (auto i = 0; i < order; ++i) {
      a(j, i + 1) = a(j, i) * x(j);
    }
  }
  auto q = a.householderQr();
  Eigen::VectorXd result = q.solve(y);
  return std::vector<double>(result.data(), result.data() + result.size());
}

} // namespace

ModelPredictiveController::ModelPredictiveController(double lf,
                                                     double dt,
                                                     size_t n_points,
                                                     size_t n_skipped_points,
                                                     double v)
  : solver_(lf, kMaxSteering, dt, n_points, n_skipped_points, v * kMphToMps) {
  // Empty.
}

// Public Methods
// -----------------------------------------------------------------------------

void ModelPredictiveController::Update(const std::vector<double>& waypoints_x,
                                       const std::vector<double>& waypoints_y,
                                       double x,
                                       double y,
                                       double yaw,
                                       double speed,
                                       ControlFunction control_function) {
  // Convert absolute to relative coordinates
  Eigen::Map<const Eigen::VectorXd> abs_x(&waypoints_x[0], waypoints_x.size());
  Eigen::Map<const Eigen::VectorXd> abs_y(&waypoints_y[0], waypoints_y.size());
  Eigen::VectorXd rel_x(waypoints_x.size());
  Eigen::VectorXd rel_y(waypoints_y.size());
  AbsoluteToRelativeCoords(abs_x, abs_y, x, y, yaw, rel_x, rel_y);

  // Get coefficients
  auto coeffs = FitPolynomial(rel_x, rel_y, kPolynomialOrder);

  // Solve the problem
  MpcSolver::State state;
  state.v = speed * kMphToMps;
  state.cte = EvaluatePolynomial(coeffs, state.x) - state.y;
  state.epsi = state.psi + std::atan(EvaluateDerivative(coeffs, state.x));
  MpcSolver::Solution solution;
  if (!solver_(state, coeffs, solution)) {
    std::cerr << "Failed to find a solution!" << std::endl;
    // Stop the vehicle
    control_function(
      0, 0,
      solution.sequence_x, solution.sequence_y,
      std::vector<double>(rel_x.data(), rel_x.data() + rel_x.size()),
      std::vector<double>(rel_y.data(), rel_y.data() + rel_y.size()));
  } else {
    // Control the vehicle
    control_function(
      -solution.sequence_delta.at(0) / kMaxSteering, solution.sequence_a.at(0),
      solution.sequence_x, solution.sequence_y,
      std::vector<double>(rel_x.data(), rel_x.data() + rel_x.size()),
      std::vector<double>(rel_y.data(), rel_y.data() + rel_y.size()));
  }
}
