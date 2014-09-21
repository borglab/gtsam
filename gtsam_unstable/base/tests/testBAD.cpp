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
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/Testable.h>

#include <boost/make_shared.hpp>

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
class BADFactor: NonlinearFactor {

public:

  /// Constructor
  BADFactor(const Expression<T>& t) {
  }

  /**
   * Calculate the error of the factor
   * This is typically equal to log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/sigma^2 \f$
   */
  double error(const Values& c) const {
    return 0;
  }

  /// get the dimension of the factor (number of rows on linearization)
  size_t dim() const {
    return 0;
  }

  /// linearize to a GaussianFactor
  boost::shared_ptr<GaussianFactor> linearize(const Values& c) const {
    return boost::shared_ptr<JacobianFactor>(new JacobianFactor());
  }

};

/* ************************************************************************* */

TEST(BAD, test) {

  // Create some values
  Values values;
  values.insert(1,Pose3());
  values.insert(2,Point3(0,0,1));
  values.insert(3,Cal3_S2());

  // Create old-style factor to create expected value and derivatives
  Point2 measured(0,1);
  SharedNoiseModel model = noiseModel::Unit::Create(2);
  GeneralSFMFactor2<Cal3_S2> old(measured, model, 1, 2, 3);
  GaussianFactor::shared_ptr expected = old.linearize(values);

  // Create leaves
  Expression<Pose3> x(1);
  Expression<Point3> p(2);
  Expression<Cal3_S2> K(3);
  Expression<Point2> uv(measured);

  // Create expression tree
  Expression<Point3> p_cam = transformTo(x, p);
  Expression<Point2> projection = project(p_cam);
  Expression<Point2> uv_hat = uncalibrate(K, projection);
  Expression<Point2> e = uv - uv_hat;

  // Create factor
  BADFactor<Point2> f(e);

  // Check value
  EXPECT_DOUBLES_EQUAL(old.error(values), f.error(values), 1e-9);

  // Check dimension
  EXPECT_LONGS_EQUAL(0, f.dim());

  // Check linearization
  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
  EXPECT( assert_equal(*expected, *gf, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

