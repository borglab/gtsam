/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBAD.cpp
 * @date September 18, 2014
 * @author Frank Dellaert
 * @brief unit tests for Block Automatic Differentiation
 */

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/inference/Key.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/// This class might have to become a class hierarchy ?
template<class T>
class Expression {

public:

  /// Constructor with a single key
  Expression(Key key) {
  }

  /// Constructor with a value, yielding a constant
  Expression(const T& t) {
  }
};

/// Expression version of transform
Expression<Point3> transformTo(const Expression<Pose3>& x,
    const Expression<Point3>& p) {
  return Expression<Point3>(0);
}

/// Expression version of project
Expression<Point2> project(const Expression<Point3>& p) {
  return Expression<Point2>(0);
}

/// Expression version of uncalibrate
Expression<Point2> uncalibrate(const Expression<Cal3_S2>& K,
    const Expression<Point2>& p) {
  return Expression<Point2>(0);
}

/// Expression version of Point2.sub
Expression<Point2> operator -(const Expression<Point2>& p,
    const Expression<Point2>& q) {
  return Expression<Point2>(0);
}

/// AD Factor
template<class T>
class BADFactor: NoiseModelFactor {

public:

  /// Constructor
  BADFactor(const Expression<T>& t) {
  }

  Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if (H) H->push_back(zeros(2,2));
    return Vector();
  }

};

/* ************************************************************************* */

TEST(BAD, test) {

  // Create leaves
  Expression<Pose3> x(1);
  Expression<Point3> p(2);
  Expression<Cal3_S2> K(3);
  Expression<Point2> uv(Point2(300, 62));

  // Create expression tree
  Expression<Point3> p_cam = transformTo(x, p);
  Expression<Point2> projection = project(p_cam);
  Expression<Point2> uv_hat = uncalibrate(K, projection);
  Expression<Point2> e = uv - uv_hat;

  // Create factor
  BADFactor<Point2> f(e);

  // evaluate, with derivatives
  Values values;
  vector<Matrix> jacobians;
  Vector actual = f.unwhitenedError(values, jacobians);

  // Check value
  Vector expected = (Vector(2) << 0, 0);
  EXPECT(assert_equal(expected, actual));

  // Check derivatives
  Matrix expectedHx = zeros(2,3);
  EXPECT(assert_equal(expectedHx, jacobians[0]));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

