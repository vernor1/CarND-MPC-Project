#include "MpcSolver.h"
#include <cppad/ipopt/solve.hpp>

enum { kStateVars = 6 };

enum { kActuators = 2 };

auto kMaxAngle = 25. / 180. * M_PI;

MpcSolver::MpcSolver(size_t n_points, double dt, double lf)
  : n_points_(n_points),
    offset_(n_points),
    dt_(dt),
    lf_(lf) {
  // Empty.
}

bool MpcSolver::operator()(const State& state,
                           const std::vector<double>& coeffs,
                           Solution& solution) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Number of independent variables: N points +  N - 1 actuations
  auto n_vars = n_points_ * kStateVars + (n_points_ - 1) * kActuators;

  // Number of constraints
  auto n_constraints = n_points_ * kStateVars;

  // Initial value of the independent variables. Should be 0 except for the
  // state values.
  Dvector vars(n_vars);
  for (auto i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[offset_.x] = state.x;
  vars[offset_.y] = state.y;
  vars[offset_.psi] = state.psi;
  vars[offset_.v] = state.v;
  vars[offset_.cte] = state.cte;
  vars[offset_.epsi] = state.epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits to the max negative and
  // positive values
  for (auto i = 0; i < offset_.delta; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25 degrees (values
  // in radians)
  for (auto i = offset_.delta; i < offset_.a; ++i) {
    // TODO: Replace with constant
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits
  for (auto i = offset_.a; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for constraints. All of these should be 0 except the
  // initial state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (auto i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[offset_.x] = state.x;
  constraints_lowerbound[offset_.y] = state.y;
  constraints_lowerbound[offset_.psi] = state.psi;
  constraints_lowerbound[offset_.v] = state.v;
  constraints_lowerbound[offset_.cte] = state.cte;
  constraints_lowerbound[offset_.epsi] = state.epsi;

  constraints_upperbound[offset_.x] = state.x;
  constraints_upperbound[offset_.y] = state.y;
  constraints_upperbound[offset_.psi] = state.psi;
  constraints_upperbound[offset_.v] = state.v;
  constraints_upperbound[offset_.cte] = state.cte;
  constraints_upperbound[offset_.epsi] = state.epsi;

  // Object that computes objective and constraints
  MpcEvaluator evaluator(n_points_, dt_, lf_, coeffs);

  // Options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage of sparse
  // routines, this makes the computation MUCH FASTER. If you can uncomment 1 of
  // these and see if it makes a difference or not but if you uncomment both the
  // computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds. Change
  // this as you see fit.
  options += "Numeric max_cpu_time          0.1\n";

  // Place to return solution
  CppAD::ipopt::solve_result<Dvector> sol;

  // Solve the problem
  CppAD::ipopt::solve<Dvector, MpcEvaluator>(options,
                                             vars,
                                             vars_lowerbound,
                                             vars_upperbound,
                                             constraints_lowerbound,
                                             constraints_upperbound,
                                             evaluator,
                                             sol);

  // Check some of the solution values
  if (sol.status == CppAD::ipopt::solve_result<Dvector>::success) {
    solution.cost = sol.obj_value;
    solution.sequence_x.assign(
      &sol.x[offset_.x], &sol.x[offset_.x] + n_points_);
    solution.sequence_y.assign(
      &sol.x[offset_.y], &sol.x[offset_.y] + n_points_);
    solution.sequence_psi.assign(
      &sol.x[offset_.psi], &sol.x[offset_.psi] + n_points_);
    solution.sequence_v.assign(
      &sol.x[offset_.v], &sol.x[offset_.v] + n_points_);
    solution.sequence_cte.assign(
      &sol.x[offset_.cte], &sol.x[offset_.cte] + n_points_);
    solution.sequence_epsi.assign(
      &sol.x[offset_.epsi], &sol.x[offset_.epsi] + n_points_);
    solution.sequence_delta.assign(
      &sol.x[offset_.delta], &sol.x[offset_.delta] + n_points_);
    solution.sequence_a.assign(
      &sol.x[offset_.a], &sol.x[offset_.a] + n_points_);
  }

  return sol.status == CppAD::ipopt::solve_result<Dvector>::success;
}
