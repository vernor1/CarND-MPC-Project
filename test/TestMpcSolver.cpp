#include "gtest/gtest.h"
#include "../src/Eigen-3.3/Eigen/Core"
#include "../src/Eigen-3.3/Eigen/QR"
#include "../src/MpcSolver.h"

// Evaluates a polynomial.
double EvaluatePolynomial(const std::vector<double>& coeffs, double x) {
  double result = 0;
  for (auto i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * std::pow(x, i);
  }
  return result;
}

// Fit a polynomial.
std::vector<double> FitPolynomial(std::vector<double> xvals,
                                  std::vector<double> yvals,
                                  int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (auto j = 0; j < xvals.size(); ++j) {
    for (auto i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals[j];
    }
  }

  auto Q = A.householderQr();
  Eigen::Map<Eigen::VectorXd> y(&yvals[0], yvals.size());
  Eigen::VectorXd result = Q.solve(y);
  return std::vector<double>(result.data(), result.data() + result.size());
}

TEST(MpcSolver, Linear) {
  size_t n = 25;
  int iters = 50;

  auto max_steering = 25. / 180. * M_PI;
  MpcSolver solver(2.67, max_steering, 0.05, n, 0, 23);

  // Define coeffs
  std::vector<double> ptsx = {-100, 100};
  std::vector<double> ptsy = {-1, -1};
  auto coeffs = FitPolynomial(ptsx, ptsy, 1);

  // Define state
  MpcSolver::State state;
  state.x = -1;
  state.y = 10;
  state.psi = 0;
  state.v = 10;
  state.cte = EvaluatePolynomial(coeffs, state.x) - state.y;
  state.epsi = state.psi - atan(coeffs[1]);

  // Declare solution
  MpcSolver::Solution solution;

  // Simulate driving along the line
  for (auto i = 0; i < iters; ++i) {
    auto result = solver(state, coeffs, solution);
    state.x = solution.sequence_x.at(1);
    state.y = solution.sequence_y.at(1),
    state.psi = solution.sequence_psi.at(1),
    state.v = solution.sequence_v.at(1),
    state.cte = solution.sequence_cte.at(1),
    state.epsi = solution.sequence_epsi.at(1);
  }

  EXPECT_NEAR(17.551, solution.sequence_x.at(1), 1e-3);
  EXPECT_NEAR(-1.697, solution.sequence_y.at(1), 1e-3);
  EXPECT_NEAR(-0.056, solution.sequence_psi.at(1), 1e-3);
  EXPECT_NEAR(9.771, solution.sequence_v.at(1), 1e-3);
  EXPECT_NEAR(0.610, solution.sequence_cte.at(1), 1e-3);
  EXPECT_NEAR(-0.089, solution.sequence_epsi.at(1), 1e-3);
  EXPECT_NEAR(0.092, solution.sequence_delta.at(0), 1e-3);
  EXPECT_NEAR(1, solution.sequence_a.at(0), 1e-3);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
