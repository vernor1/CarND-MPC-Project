#ifndef MPCEVALUATOR_H
#define MPCEVALUATOR_H

#include <cppad/cppad.hpp>

// Implements the MPC Evaluator for plugging it into the Ipopt optimizer.
class MpcEvaluator {
public:
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

  // Contains the offset of every state variable and actuator.
  struct VariableOffset {
    size_t x;
    size_t y;
    size_t psi;
    size_t v;
    size_t cte;
    size_t epsi;
    size_t delta;
    size_t a;
    VariableOffset(size_t n_points)
    : x(),
      y(x + n_points),
      psi(y + n_points),
      v(psi + n_points),
      cte(v + n_points),
      epsi(cte + n_points),
      delta(epsi + n_points),
      a(delta + n_points - 1) { }
  };

  // Contructor.
  // @param n_points  Number of points in the predicted path.
  // @param dt        Time difference between predicted waypoints.
  // @param lf        Distance between the front of the vehicle and its center
  //                  of gravity.
  // @param v         Desired velocity in [mph].
  // @param coeffs    Polynomial coefficients of the refernce path.
  MpcEvaluator(size_t n_points,
               double dt,
               double lf,
               double v,
               const std::vector<double>& coeffs);

  // Destructor.
  virtual ~MpcEvaluator() { }

  // Evaluates the solution.
  // @param[out] fg    CppAD-vector of the evaluation.
  // @param[in]  vars  CppAD-vector of state variables and actuations
  void operator()(ADvector& fg, const ADvector& vars) const;

private:
  size_t n_points_;
  VariableOffset offset_;
  double dt_;
  double lf_;
  double v_;
  std::vector<double> coeffs_;
};

#endif // MPCEVALUATOR_H
