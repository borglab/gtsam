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
#include <gtsam/basis/polynomial/IrwinHall.h>


using namespace gtsam;
using namespace gtsam::kernels;

// The summation of so many terms does get noisy,
// so split the epsilon and the tolerance
double epsilon = 1e-9;
double tolerance = 1e-7;




// test boundary conditions for cumulative distribution
TEST( IrwinHallCDF , Boundaries1 ) {
  auto poly = IrwinHallCDF1;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.get_intervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 1.0, poly.evaluate(poly.get_intervals().back()), tolerance);
}
TEST( IrwinHallCDF , Boundaries2 ) {
  auto poly = IrwinHallCDF2;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.get_intervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 1.0, poly.evaluate(poly.get_intervals().back()), tolerance);
}
TEST( IrwinHallCDF , Boundaries3 ) {
  auto poly = IrwinHallCDF3;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.get_intervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 1.0, poly.evaluate(poly.get_intervals().back()), tolerance);
}
TEST( IrwinHallCDF , Boundaries4 ) {
  auto poly = IrwinHallCDF4;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.get_intervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 1.0, poly.evaluate(poly.get_intervals().back()), tolerance);
}
TEST( IrwinHallCDF , Boundaries5 ) {
  auto poly = IrwinHallCDF5;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.get_intervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 1.0, poly.evaluate(poly.get_intervals().back()), tolerance);
}
TEST( IrwinHallCDF , Boundaries6 ) {
  auto poly = IrwinHallCDF6;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.get_intervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 1.0, poly.evaluate(poly.get_intervals().back()), tolerance);
}




// test continuity where the polynomial pieces join
TEST( IrwinHallCDF , Continuity1 ) {
  auto poly = IrwinHallCDF1;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHallCDF , Continuity2 ) {
  auto poly = IrwinHallCDF2;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHallCDF , Continuity3 ) {
  auto poly = IrwinHallCDF3;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHallCDF , Continuity4 ) {
  auto poly = IrwinHallCDF4;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHallCDF , Continuity5 ) {
  auto poly = IrwinHallCDF5;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHallCDF , Continuity6 ) {
  auto poly = IrwinHallCDF6;
  for(const double &t : poly.get_intervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}


// test continuity of derivatives where the polynomial pieces join
TEST( IrwinHallCDF , DerivativeContinuity1 ) {
  auto poly = IrwinHallCDF1;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHallCDF , DerivativeContinuity2 ) {
  auto poly = IrwinHallCDF2;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHallCDF , DerivativeContinuity3 ) {
  auto poly = IrwinHallCDF3;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHallCDF , DerivativeContinuity4 ) {
  auto poly = IrwinHallCDF4;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHallCDF , DerivativeContinuity5 ) {
  auto poly = IrwinHallCDF5;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHallCDF , DerivativeContinuity6 ) {
  auto poly = IrwinHallCDF6;
  int max_d = poly.order-1;
  for(const double &t : poly.get_intervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluate_d(d, t+epsilon),
                            poly.evaluate_d(d, t-epsilon), tolerance );
    }
  }
}

// TODO test the distribution is scaled sensibly against the CDF





/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

