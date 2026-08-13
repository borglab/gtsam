/**
 * @file testPiecewisePolynomial.cpp
 * @brief Validate analytic Jacobians of piecewise polynomial kernels.
 * @author Brett Downing
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/basis/IrwinHall.h>

#include <cmath>

using namespace gtsam;
using namespace gtsam::kernels;

/* ************************************************************************* */
namespace piecewise_polynomial {

constexpr double kTolerance = 1e-7;

template <class Polynomial>
bool checkDerivativeJacobians(const Polynomial& polynomial) {
  const int maximumDerivative = static_cast<int>(polynomial.order) - 2;
  for (double t : polynomial.getIntervals()) {
    for (int derivative = 0; derivative < maximumDerivative; ++derivative) {
      Eigen::Matrix<double, 1, 1> jacobian;
      polynomial.evaluateDerivative(derivative, t, jacobian);
      const double expected = polynomial.evaluateDerivative(derivative + 1, t);
      if (std::abs(expected - jacobian(0, 0)) > kTolerance) return false;
    }
  }
  return true;
}

// Verifies a PDF reports its next derivative through the Jacobian argument.
TEST(PiecewisePolynomial, PdfDerivativeJacobian) {
  EXPECT(checkDerivativeJacobians(IrwinHall6));
}

// Verifies a CDF reports its next derivative through the Jacobian argument.
TEST(PiecewisePolynomial, CdfDerivativeJacobian) {
  EXPECT(checkDerivativeJacobians(IrwinHallCDF6));
}

}  // namespace piecewise_polynomial
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
