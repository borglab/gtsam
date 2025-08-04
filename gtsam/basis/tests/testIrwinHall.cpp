/**
 * @file    TestIrwinHall.cpp
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
#include <gtsam/basis/polynomial/kernels.h>
#include <gtsam/basis/polynomial/IrwinHall.h>


using namespace gtsam;
using namespace gtsam::kernels;

// The summation of so many terms does get noisy,
// so split the epsilon and the tolerance
double epsilon = 1e-9;
double tolerance = 1e-7;

// test continuity where the polynomial pieces join
TEST( IrwinHall , Continuity1 ) {
  auto poly = IrwinHall1;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHall , Continuity2 ) {
  auto poly = IrwinHall2;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHall , Continuity3 ) {
  auto poly = IrwinHall3;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHall , Continuity4 ) {
  auto poly = IrwinHall4;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHall , Continuity5 ) {
  auto poly = IrwinHall5;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHall , Continuity6 ) {
  auto poly = IrwinHall6;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}


// test continuity of derivatives where the polynomial pieces join
TEST( IrwinHall , DerivativeContinuity1 ) {
  auto poly = IrwinHall1;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHall , DerivativeContinuity2 ) {
  auto poly = IrwinHall2;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHall , DerivativeContinuity3 ) {
  auto poly = IrwinHall3;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHall , DerivativeContinuity4 ) {
  auto poly = IrwinHall4;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHall , DerivativeContinuity5 ) {
  auto poly = IrwinHall5;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHall , DerivativeContinuity6 ) {
  auto poly = IrwinHall6;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}


// test jacobian of evaluate against the next derivative
TEST( IrwinHall , DerivativeIsJacobian ) {
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


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

