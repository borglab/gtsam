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

// these tests look for curve continuity across control-flow boundaries
// tolerance should be about 2*(maximum gradient)*epsilon
// actual polynomial construction mistakes will be extremely large
double epsilon = 1e-7;
double tolerance = 100*epsilon;



// test boundary conditions at the limits of the domain
TEST( poly , Boundaries1 ) {
  auto poly = IrwinHall1;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().back()), tolerance);
}
TEST( poly , Boundaries2 ) {
  auto poly = IrwinHall2;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().back()), tolerance);
}
TEST( poly , Boundaries3 ) {
  auto poly = IrwinHall3;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().back()), tolerance);
}
TEST( poly , Boundaries4 ) {
  auto poly = IrwinHall4;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().back()), tolerance);
}
TEST( poly , Boundaries5 ) {
  auto poly = IrwinHall5;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().back()), tolerance);
}
TEST( poly , Boundaries6 ) {
  auto poly = IrwinHall6;
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().front()), tolerance);
  EXPECT_DOUBLES_EQUAL( 0.0, poly.evaluate(poly.getIntervals().back()), tolerance);
}



// test continuity where the polynomial pieces join
TEST( IrwinHall , Continuity1 ) {
  auto poly = IrwinHall1;
  for(const double &t : poly.getIntervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHall , Continuity2 ) {
  auto poly = IrwinHall2;
  for(const double &t : poly.getIntervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHall , Continuity3 ) {
  auto poly = IrwinHall3;
  for(const double &t : poly.getIntervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHall , Continuity4 ) {
  auto poly = IrwinHall4;
  for(const double &t : poly.getIntervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHall , Continuity5 ) {
  auto poly = IrwinHall5;
  for(const double &t : poly.getIntervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}
TEST( IrwinHall , Continuity6 ) {
  auto poly = IrwinHall6;
  for(const double &t : poly.getIntervals()) {
    EXPECT_DOUBLES_EQUAL( poly.evaluate(t+epsilon),
                          poly.evaluate(t-epsilon), tolerance );
  }
}


// test continuity of derivatives where the polynomial pieces join
TEST( IrwinHall , DerivativeContinuity1 ) {
  auto poly = IrwinHall1;
  int max_d = poly.order-1;
  for(const double &t : poly.getIntervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluateDerivative(d, t+epsilon),
                            poly.evaluateDerivative(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHall , DerivativeContinuity2 ) {
  auto poly = IrwinHall2;
  int max_d = poly.order-1;
  for(const double &t : poly.getIntervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluateDerivative(d, t+epsilon),
                            poly.evaluateDerivative(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHall , DerivativeContinuity3 ) {
  auto poly = IrwinHall3;
  int max_d = poly.order-1;
  for(const double &t : poly.getIntervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluateDerivative(d, t+epsilon),
                            poly.evaluateDerivative(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHall , DerivativeContinuity4 ) {
  auto poly = IrwinHall4;
  int max_d = poly.order-1;
  for(const double &t : poly.getIntervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluateDerivative(d, t+epsilon),
                            poly.evaluateDerivative(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHall , DerivativeContinuity5 ) {
  auto poly = IrwinHall5;
  int max_d = poly.order-1;
  for(const double &t : poly.getIntervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluateDerivative(d, t+epsilon),
                            poly.evaluateDerivative(d, t-epsilon), tolerance );
    }
  }
}
TEST( IrwinHall , DerivativeContinuity6 ) {
  auto poly = IrwinHall6;
  int max_d = poly.order-1;
  for(const double &t : poly.getIntervals()) {
    for(int d=0; d<max_d; d++) {
      EXPECT_DOUBLES_EQUAL( poly.evaluateDerivative(d, t+epsilon),
                            poly.evaluateDerivative(d, t-epsilon), tolerance );
    }
  }
}






/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

