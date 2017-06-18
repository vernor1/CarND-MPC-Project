#ifndef MPCSOLVER_H
#define MPCSOLVER_H

#include "MpcEvaluator.h"

class MpcSolver {
public:
  struct State {
    double x;
    double y;
    double psi;
    double v;
    double cte;
    double epsi;
    State() : x(), y(), psi(), v(), cte(), epsi() { }
  };

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
  MpcSolver(double lf,
            double max_steering,
            double dt,
            size_t n_points,
            size_t s_points,
            double v);

  // Destructor.
  virtual ~MpcSolver() { }

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
