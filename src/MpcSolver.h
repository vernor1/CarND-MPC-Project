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
    // TODO: Add default ctor
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
  MpcSolver(size_t n_points, double dt, double lf);

  // Destructor.
  virtual ~MpcSolver() { }

  bool operator()(const State& state,
                  const std::vector<double>& coeffs,
                  Solution& solution);

private:
  size_t n_points_;
  MpcEvaluator::VariableOffset offset_;
  double dt_;
  double lf_;
};

#endif // MPCSOLVER_H
