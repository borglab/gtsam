/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testDistanceFactor.cpp
 *  @brief Unit tests for DistanceFactor Class
 *  @author Duy-Nguyen Ta
 *  @date Oct 2012
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/DistanceFactor.h>

using namespace std;
using namespace gtsam;

typedef DistanceFactor<Point2> DistanceFactor2D;
typedef DistanceFactor<Point3> DistanceFactor3D;

SharedDiagonal noise = noiseModel::Unit::Create(1);
Point3 P(0., 1., 2.5), Q(10., -81., 7.);
Point2 p(1., 2.5), q(-81., 7.);

/* ************************************************************************* */
TEST(DistanceFactor, Point3) {
  DistanceFactor3D distanceFactor(0, 1, P.distance(Q), noise);
  Matrix H1, H2;
  Vector error = distanceFactor.evaluateError(P, Q, H1, H2);

  Vector expectedError = zero(1);
  EXPECT(assert_equal(expectedError, error, 1e-10));

  boost::function<Vector(const Point3&, const Point3&)> testEvaluateError(
      boost::bind(&DistanceFactor3D::evaluateError, distanceFactor, _1, _2,
          boost::none, boost::none));
  Matrix numericalH1 = numericalDerivative21(testEvaluateError, P, Q, 1e-5);
  Matrix numericalH2 = numericalDerivative22(testEvaluateError, P, Q, 1e-5);

  EXPECT(assert_equal(numericalH1, H1, 1e-8));
  EXPECT(assert_equal(numericalH2, H2, 1e-8));

}

/* ************************************************************************* */
TEST(DistanceFactor, Point2) {
  DistanceFactor2D distanceFactor(0, 1, p.distance(q), noise);
  Matrix H1, H2;
  Vector error = distanceFactor.evaluateError(p, q, H1, H2);

  Vector expectedError = zero(1);
  EXPECT(assert_equal(expectedError, error, 1e-10));

  boost::function<Vector(const Point2&, const Point2&)> testEvaluateError(
      boost::bind(&DistanceFactor2D::evaluateError, distanceFactor, _1, _2,
          boost::none, boost::none));
  Matrix numericalH1 = numericalDerivative21(testEvaluateError, p, q, 1e-5);
  Matrix numericalH2 = numericalDerivative22(testEvaluateError, p, q, 1e-5);

  EXPECT(assert_equal(numericalH1, H1, 1e-8));
  EXPECT(assert_equal(numericalH2, H2, 1e-8));

}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

