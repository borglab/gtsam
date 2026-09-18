/**
 * @file testIrwinHallCDF.cpp
 * @brief Validate Irwin-Hall CDF coefficients and continuity.
 * @author Brett Downing
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/basis/IrwinHall.h>

#include <cmath>

using namespace gtsam;
using namespace gtsam::kernels;

/* ************************************************************************* */
namespace irwin_hall_cdf {

constexpr double kEpsilon = 1e-9;
constexpr double kTolerance = 1e-7;

template <class Polynomial>
bool checkBoundaries(const Polynomial& polynomial) {
  return std::abs(polynomial.evaluate(polynomial.getIntervals().front())) <=
             kTolerance &&
         std::abs(polynomial.evaluate(polynomial.getIntervals().back()) -
                  1.0) <= kTolerance;
}

template <class Polynomial>
bool checkContinuity(const Polynomial& polynomial) {
  for (double t : polynomial.getIntervals()) {
    if (std::abs(polynomial.evaluate(t + kEpsilon) -
                 polynomial.evaluate(t - kEpsilon)) > kTolerance) {
      return false;
    }
  }
  return true;
}

template <class Polynomial>
bool checkDerivativeContinuity(const Polynomial& polynomial) {
  const int maximumDerivative = static_cast<int>(polynomial.order) - 1;
  for (double t : polynomial.getIntervals()) {
    for (int derivative = 0; derivative < maximumDerivative; ++derivative) {
      if (std::abs(polynomial.evaluateDerivative(derivative, t + kEpsilon) -
                   polynomial.evaluateDerivative(derivative, t - kEpsilon)) >
          kTolerance) {
        return false;
      }
    }
  }
  return true;
}

// Verifies every CDF spans zero to one over its support.
TEST(IrwinHallCDF, Boundaries) {
  EXPECT(checkBoundaries(IrwinHallCDF1));
  EXPECT(checkBoundaries(IrwinHallCDF2));
  EXPECT(checkBoundaries(IrwinHallCDF3));
  EXPECT(checkBoundaries(IrwinHallCDF4));
  EXPECT(checkBoundaries(IrwinHallCDF5));
  EXPECT(checkBoundaries(IrwinHallCDF6));
}

// Verifies every CDF is continuous where its polynomial pieces meet.
TEST(IrwinHallCDF, Continuity) {
  EXPECT(checkContinuity(IrwinHallCDF1));
  EXPECT(checkContinuity(IrwinHallCDF2));
  EXPECT(checkContinuity(IrwinHallCDF3));
  EXPECT(checkContinuity(IrwinHallCDF4));
  EXPECT(checkContinuity(IrwinHallCDF5));
  EXPECT(checkContinuity(IrwinHallCDF6));
}

// Verifies all meaningful CDF derivatives are continuous across pieces.
TEST(IrwinHallCDF, DerivativeContinuity) {
  EXPECT(checkDerivativeContinuity(IrwinHallCDF1));
  EXPECT(checkDerivativeContinuity(IrwinHallCDF2));
  EXPECT(checkDerivativeContinuity(IrwinHallCDF3));
  EXPECT(checkDerivativeContinuity(IrwinHallCDF4));
  EXPECT(checkDerivativeContinuity(IrwinHallCDF5));
  EXPECT(checkDerivativeContinuity(IrwinHallCDF6));
}

}  // namespace irwin_hall_cdf
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
