#ifndef MPCSOLVER_H
#define MPCSOLVER_H

#include "MpcEvaluator.h"

// Implements the MPC Solver which given the current vehicle state and
// coefficients of the refence path polynomia, produces the predicted actuations
// and states for a desired duration and frequency. Instantiates an MPC
// Evaluator object for solving the problem with the Ipopt optimizer.
class MpcSolver {
public:
  // Contains an initial state of the kinematic vehicle model.
  struct State {
    double x;
    double y;
    double psi;
    double v;
    double cte;
    double epsi;
    State() : x(), y(), psi(), v(), cte(), epsi() { }
  };

  // Contains a solution.
  struct Solution {
    double cost;
    std::vector<double> sequence_x;
    std::vector<double> sequence_y;
    std::vector<double> sequence_psi;
    std::vector<double> sequence_v;
    std::vector<double> sequence_cte;
    std::vector<double> sequence_epsi;
    std::vector<double> sequence_delta;
    std::vector<double> sequence_a;
  };

  // Contructor.
  // @param lf        Distance between the front of the vehicle and its center
  //                  of gravity.
  // @param max_steering  Max steering angle of the vehicle in [rad].
  // @param dt        Time difference between predicted waypoints.
  // @param n_points  Number of points in the predicted path.
  // @param s_points  Number of nearest predicted waypoints to be skipped
  //                  because of latency.
  // @param v         Desired velocity in [mph].
  MpcSolver(double lf,
            double max_steering,
            double dt,
            size_t n_points,
            size_t s_points,
            double v);

  // Destructor.
  virtual ~MpcSolver() { }

  // Computes the solution.
  // @param[in]  state     Initial model state.
  // @param[in]  coeffs    Polynomial coefficients of the refernce path.
  // @param[out] solution  Solution of the problem.
  // @return               True if a solution is found, false otherwise.
  bool operator()(const State& state,
                  const std::vector<double>& coeffs,
                  Solution& solution);

private:
  double lf_;
  double max_steering_;
  double dt_;
  size_t n_points_;
  size_t s_points_;
  double v_;
  MpcEvaluator::VariableOffset offset_;
  double current_delta_;
  double current_a_;
};

#endif // MPCSOLVER_H
