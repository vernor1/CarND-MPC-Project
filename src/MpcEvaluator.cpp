#include "MpcEvaluator.h"

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// Cost multiplier to penalize a high difference between subsequent delta
// values.
const auto kDeltaDiffPenalty = 1e4;
// Local Helper-Functions
// -----------------------------------------------------------------------------

// Evaluates a polynomial.
// @param[in] coeffs  Polynomial coefficients.
// @param[in] x       CppAD-argument to avaluate the polynomial for.
// @return            CppAD-value of the function.
CppAD::AD<double> EvaluatePolynomial(const std::vector<double>& coeffs,
                                     const CppAD::AD<double>& x) {
  CppAD::AD<double> result = 0;
  for (auto i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

// Evaluates the 1st derivative of a polynomial.
// @param[in] coeffs  Polynomial coefficients.
// @param[in] x       CppAD-argument to avaluate the derivative for.
// @return            CppAD-value of the function derivative.
CppAD::AD<double> EvaluateDerivative(const std::vector<double>& coeffs,
                                     const CppAD::AD<double>& x) {
  auto n_coeffs = coeffs.size();
  assert(n_coeffs > 0);
  CppAD::AD<double> result = 0;
  for (auto i = 1; i < n_coeffs; ++i) {
    result += i * coeffs[i] * CppAD::pow(x, i - 1);
  }
  return result;
}

} // namespace

// Public Methods
// -----------------------------------------------------------------------------

MpcEvaluator::MpcEvaluator(size_t n_points,
                           double dt,
                           double lf,
                           double v,
                           const std::vector<double>& coeffs)
  : n_points_(n_points),
    offset_(n_points),
    dt_(dt),
    lf_(lf),
    v_(v),
    coeffs_(coeffs) {
  // Empty.
}

void MpcEvaluator::operator()(ADvector& fg, const ADvector& vars) const {
  // Initial cost is 0
  fg[0] = 0;

  // Penaltize error and speed values
  auto cte = vars[offset_.cte];
  for (auto t = 0; t < n_points_; ++t) {
    fg[0] += CppAD::pow(vars[offset_.cte + t], 2);
    fg[0] += CppAD::pow(vars[offset_.epsi + t], 2);
    fg[0] += CppAD::pow(vars[offset_.v + t] - v_ + CppAD::pow(cte, 2), 2);
  }

  // Penalize high actuation values
  for (auto t = 0; t < n_points_ - 1; ++t) {
    fg[0] += CppAD::pow(vars[offset_.delta + t], 2);
    fg[0] += CppAD::pow(vars[offset_.a + t], 2);
  }

  // Penalize high difference between subsequent actuation values
  for (auto t = 0; t < n_points_ - 2; ++t) {
    fg[0] += kDeltaDiffPenalty
      * CppAD::pow(vars[offset_.delta + t + 1] - vars[offset_.delta + t], 2);
    fg[0] += CppAD::pow(vars[offset_.a + t + 1] - vars[offset_.a + t], 2);
  }

  // Initial constraints
  fg[1 + offset_.x] = vars[offset_.x];
  fg[1 + offset_.y] = vars[offset_.y];
  fg[1 + offset_.psi] = vars[offset_.psi];
  fg[1 + offset_.v] = vars[offset_.v];
  fg[1 + offset_.cte] = vars[offset_.cte];
  fg[1 + offset_.epsi] = vars[offset_.epsi];

  // The rest of the constraints
  for (auto t = 1; t < n_points_; ++t) {
    // x<t>
    CppAD::AD<double> x0 = vars[offset_.x + t - 1];
    // y<t>
    CppAD::AD<double> y0 = vars[offset_.y + t - 1];
    // psi<t>
    CppAD::AD<double> psi0 = vars[offset_.psi + t - 1];
    // v<t>
    CppAD::AD<double> v0 = vars[offset_.v + t - 1];
    // epsi<t>
    CppAD::AD<double> epsi0 = vars[offset_.epsi + t - 1];
    // delta<t>
    CppAD::AD<double> delta0 = vars[offset_.delta + t - 1];
    // a<t>
    CppAD::AD<double> a0 = vars[offset_.a + t - 1];
    // x<t+1>
    CppAD::AD<double> x1 = vars[offset_.x + t];
    // y<t+1>
    CppAD::AD<double> y1 = vars[offset_.y + t];
    // psi<t+1>
    CppAD::AD<double> psi1 = vars[offset_.psi + t];
    // v<t+1>
    CppAD::AD<double> v1 = vars[offset_.v + t];
    // cte<t+1>
    CppAD::AD<double> cte1 = vars[offset_.cte + t];
    // epsi<t+1>
    CppAD::AD<double> epsi1 = vars[offset_.epsi + t];

    // x<t+1> = x<t> + v<t> * cos(psi<t>) * dt
    fg[1 + offset_.x + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt_);

    // y<t+1> = y<t> + v<t> * sin(psi<t>) * dt
    fg[1 + offset_.y + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt_);

    // psi<t+1> = psi<t> + v<t> / Lf * delta<t> * dt
    fg[1 + offset_.psi + t] = psi1 - (psi0 + v0 / lf_ * delta0 * dt_);

    // v<t+1> = v<t> + a<t> * dt
    fg[1 + offset_.v + t] = v1 - (v0 + a0 * dt_);

    // cte<t+1> = f(x<t>) - y<t> + v<t> * sin(epsi<t>) * dt
    fg[1 + offset_.cte + t] = cte1 - (EvaluatePolynomial(coeffs_, x0)
      - y0 + v0 * CppAD::sin(epsi0) * dt_);

    // epsi<t+1> = psi<t> - arctan(f'(x<t>)) + v<t> / Lf * delta<t> * dt
    fg[1 + offset_.epsi + t] = epsi1 - (psi0
      - CppAD::atan(EvaluateDerivative(coeffs_, x0) + v0 / lf_ * delta0 * dt_));
  }
}
