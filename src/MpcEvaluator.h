#ifndef MPCEVALUATOR_H
#define MPCEVALUATOR_H

#include <cppad/cppad.hpp>

class MpcEvaluator {
public:
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

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
  MpcEvaluator(size_t n_points,
               double dt,
               double lf,
               double v,
               const std::vector<double>& coeffs);

  // Destructor.
  virtual ~MpcEvaluator() { }

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
