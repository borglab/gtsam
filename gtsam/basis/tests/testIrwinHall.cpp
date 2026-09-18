/**
 * @file testIrwinHall.cpp
 * @brief Validate Irwin-Hall PDF coefficients and continuity.
 * @author Brett Downing
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/basis/IrwinHall.h>

#include <cmath>

using namespace gtsam;
using namespace gtsam::kernels;

/* ************************************************************************* */
namespace irwin_hall_pdf {

constexpr double kEpsilon = 1e-7;
constexpr double kTolerance = 100.0 * kEpsilon;

template <class Polynomial>
bool checkBoundaries(const Polynomial& polynomial) {
  return std::abs(polynomial.evaluate(polynomial.getIntervals().front())) <=
             kTolerance &&
         std::abs(polynomial.evaluate(polynomial.getIntervals().back())) <=
             kTolerance;
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

// Verifies every PDF vanishes at both support boundaries.
TEST(IrwinHall, Boundaries) {
  EXPECT(checkBoundaries(IrwinHall1));
  EXPECT(checkBoundaries(IrwinHall2));
  EXPECT(checkBoundaries(IrwinHall3));
  EXPECT(checkBoundaries(IrwinHall4));
  EXPECT(checkBoundaries(IrwinHall5));
  EXPECT(checkBoundaries(IrwinHall6));
}

// Verifies every PDF is continuous where its polynomial pieces meet.
TEST(IrwinHall, Continuity) {
  EXPECT(checkContinuity(IrwinHall1));
  EXPECT(checkContinuity(IrwinHall2));
  EXPECT(checkContinuity(IrwinHall3));
  EXPECT(checkContinuity(IrwinHall4));
  EXPECT(checkContinuity(IrwinHall5));
  EXPECT(checkContinuity(IrwinHall6));
}

// Verifies all meaningful PDF derivatives are continuous across pieces.
TEST(IrwinHall, DerivativeContinuity) {
  EXPECT(checkDerivativeContinuity(IrwinHall1));
  EXPECT(checkDerivativeContinuity(IrwinHall2));
  EXPECT(checkDerivativeContinuity(IrwinHall3));
  EXPECT(checkDerivativeContinuity(IrwinHall4));
  EXPECT(checkDerivativeContinuity(IrwinHall5));
  EXPECT(checkDerivativeContinuity(IrwinHall6));
}

}  // namespace irwin_hall_pdf
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
