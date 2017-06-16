#include "gtest/gtest.h"
#include "../src/MpcEvaluator.h"

TEST(MpcEvaluator, FirstPoint) {
  std::vector<double> coeffs = {-1, 0};
  size_t n = 5;
  size_t n_vars = n * 6 + (n - 1) * 2;
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
  ADvector vars(n_vars);
  for (auto i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  MpcEvaluator evaluator(n, 0.1, 2.67, coeffs);
  vars[0 * n] = -1;
  vars[1 * n] = 10;
  vars[2 * n] = 0;
  vars[3 * n] = 10;
  vars[4 * n] = -11;
  vars[5 * n - 1] = 0;
  ADvector fg(n_vars);
  evaluator(fg, vars);
  EXPECT_NEAR(7421, CppAD::Value(fg[0]), 1e-9);
  EXPECT_NEAR(-1, CppAD::Value(fg[1]), 1e-9);
  EXPECT_NEAR(10, CppAD::Value(fg[6]), 1e-9);
  EXPECT_NEAR(-10, CppAD::Value(fg[7]), 1e-9);
  EXPECT_NEAR(10, CppAD::Value(fg[16]), 1e-9);
  EXPECT_NEAR(-10, CppAD::Value(fg[17]), 1e-9);
  EXPECT_NEAR(-11, CppAD::Value(fg[21]), 1e-9);
  EXPECT_NEAR(11, CppAD::Value(fg[22]), 1e-9);
  EXPECT_NEAR(1, CppAD::Value(fg[23]), 1e-9);
  EXPECT_NEAR(1, CppAD::Value(fg[24]), 1e-9);
  EXPECT_NEAR(1, CppAD::Value(fg[25]), 1e-9);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
