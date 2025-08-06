/**
 * @file    TestPiecewisePolynomial.cpp
 * @brief   validate Irwin Hall coefficients and derivatives
 * @author  Brett Downing
 * @date    August 2025
 */



#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/basis/polynomial/PiecewisePolynomial.h>
#include <gtsam/basis/polynomial/IrwinHall.h>


using namespace gtsam;
using namespace gtsam::kernels;

// The summation of so many terms does get noisy,
// so split the epsilon and the tolerance
double epsilon = 1e-9;
double tolerance = 1e-7;




// test the jacobian of evaluate is the same as evaluate at the next derivative, for every valid derivative
TEST( IrwinHall , DerivativeIsJacobian1 ) {
  auto poly = IrwinHall6;
  int max_d = poly.order-2;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      Eigen::Matrix<double, 1,1> jacobian;
      poly.evaluate_d(d, t, &jacobian);
      double derivative = poly.evaluate_d(d+1, t);
      EXPECT_DOUBLES_EQUAL(jacobian[0], derivative, tolerance );
    }
  }
}

// test jacobian of evaluate against the next derivative
TEST( IrwinHallCDF , DerivativeIsJacobian2 ) {
  auto poly = IrwinHallCDF6;
  int max_d = poly.order-2;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      Eigen::Matrix<double, 1,1> jacobian;
      poly.evaluate_d(d, t, &jacobian);
      double derivative = poly.evaluate_d(d+1, t);
      EXPECT_DOUBLES_EQUAL(jacobian[0], derivative, tolerance );
    }
  }
}



/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

