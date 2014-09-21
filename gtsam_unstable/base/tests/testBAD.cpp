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
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>

#include <CppUnitLite/TestHarness.h>

namespace gtsam {

//-----------------------------------------------------------------------------
/// Constant Expression
template<class T>
class ConstantExpression {

  T value_;

public:

  typedef T type;

  /// Constructor with a value, yielding a constant
  ConstantExpression(const T& value) :
      value_(value) {
  }

  /// The value is just the stored constant
  T value(const Values& values) const {
    return value_;
  }

  /// A constant does not have a Jacobian
  std::map<Key, Matrix> jacobians(const Values& values) const {
    std::map<Key, Matrix> terms;
    return terms;
  }
};

//-----------------------------------------------------------------------------
/// Leaf Expression
template<class T>
class LeafExpression {

  Key key_;

public:

  typedef T type;

  /// Constructor with a single key
  LeafExpression(Key key) :
      key_(key) {
  }

  /// The value of a leaf is just a lookup in values
  T value(const Values& values) const {
    return values.at<T>(key_);
  }

  /// There is only a single identity Jacobian in a leaf
  std::map<Key, Matrix> jacobians(const Values& values) const {
    std::map<Key, Matrix> terms;
    const T& value = values.at<T>(key_);
    terms[key_] = eye(value.dim());
    return terms;
  }
};

//-----------------------------------------------------------------------------
/// Unary Expression
template<class T, class E>
class UnaryExpression {

public:

  typedef T (*function)(const typename E::type&);

private:

  const E expression_;
  function f_;

public:

  typedef T type;

  /// Constructor with a single key
  UnaryExpression(function f, const E& expression) :
      expression_(expression), f_(f) {
  }

  /// Evaluate the values of the subtree and call unary function on result
  T value(const Values& values) const {
    return f_(expression_.value(values));
  }

  /// Get the Jacobians from the subtree and apply the chain rule
  std::map<Key, Matrix> jacobians(const Values& values) const {
    std::map<Key, Matrix> terms = expression_.jacobians(values);
    return terms;
  }
};

//-----------------------------------------------------------------------------
/// Binary Expression
template<class T, class E1, class E2>
class BinaryExpression {

public:

  typedef T (*function)(const typename E1::type&, const typename E2::type&);

private:

  const E1 expression1_;
  const E2 expression2_;
  function f_;

public:

  typedef T type;

  /// Constructor with a single key
  BinaryExpression(function f, const E1& expression1, const E2& expression2) :
      expression1_(expression1), expression2_(expression2), f_(f) {
  }

  /// Evaluate the values of the subtrees and call binary function on result
  T value(const Values& values) const {
    return f_(expression1_.value(values), expression2_.value(values));
  }

  /// Get the Jacobians from the subtrees and apply the chain rule
  std::map<Key, Matrix> jacobians(const Values& values) const {
    std::map<Key, Matrix> terms1 = expression1_.jacobians(values);
    std::map<Key, Matrix> terms2 = expression2_.jacobians(values);
    return terms2;
  }
};

//-----------------------------------------------------------------------------

void printPair(std::pair<Key,Matrix> pair) {
  std::cout << pair.first << ": " << pair.second << std::endl;
}

//-----------------------------------------------------------------------------
/// AD Factor
template<class T, class E>
class BADFactor: NonlinearFactor {

  const T measurement_;
  const E expression_;

  /// get value from expression and calculate error with respect to measurement
  Vector unwhitenedError(const Values& values) const {
    const T& value = expression_.value(values);
    return measurement_.localCoordinates(value);
  }

public:

  /// Constructor
  BADFactor(const T& measurement, const E& expression) :
      measurement_(measurement), expression_(expression) {
  }

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   */
  virtual double error(const Values& values) const {
    if (this->active(values)) {
      const Vector e = unwhitenedError(values);
      return 0.5 * e.squaredNorm();
    } else {
      return 0.0;
    }
  }

  /// get the dimension of the factor (number of rows on linearization)
  size_t dim() const {
    return 0;
  }

  /// linearize to a GaussianFactor
  boost::shared_ptr<GaussianFactor> linearize(const Values& values) const {
    // We will construct an n-ary factor below, where  terms is a container whose
    // value type is std::pair<Key, Matrix>, specifying the
    // collection of keys and matrices making up the factor.
    std::map<Key, Matrix> terms = expression_.jacobians(values);
    std::for_each(terms.begin(), terms.end(),printPair);
    Vector b = unwhitenedError(values);
    SharedDiagonal model = SharedDiagonal();
    return boost::shared_ptr<JacobianFactor>(
        new JacobianFactor(terms, b, model));
  }

};
}

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

Point3 transformTo(const Pose3& x, const Point3& p) {
  return x.transform_to(p);
}

Point2 project(const Point3& p) {
  return PinholeCamera<Cal3_S2>::project_to_camera(p);
}

template<class CAL>
Point2 uncalibrate(const CAL& K, const Point2& p) {
  return K.uncalibrate(p);
}

/* ************************************************************************* */

TEST(BAD, test) {

  // Create some values
  Values values;
  values.insert(1, Pose3());
  values.insert(2, Point3(0, 0, 1));
  values.insert(3, Cal3_S2());

  // Create old-style factor to create expected value and derivatives
  Point2 measured(-17, 30);
  SharedNoiseModel model = noiseModel::Unit::Create(2);
  GeneralSFMFactor2<Cal3_S2> old(measured, model, 1, 2, 3);
  double expected_error = old.error(values);
  GaussianFactor::shared_ptr expected = old.linearize(values);

  // Create leaves
  LeafExpression<Pose3> x(1);
  LeafExpression<Point3> p(2);
  LeafExpression<Cal3_S2> K(3);

  // Create expression tree
  typedef BinaryExpression<Point3, LeafExpression<Pose3>, LeafExpression<Point3> > Binary1;
  Binary1 p_cam(transformTo, x, p);

  typedef UnaryExpression<Point2, Binary1> Unary1;
  Unary1 projection(project, p_cam);

  typedef BinaryExpression<Point2, LeafExpression<Cal3_S2>, Unary1> Binary2;
  Binary2 uv_hat(uncalibrate, K, projection);

  // Create factor
  BADFactor<Point2, Binary2> f(measured, uv_hat);

  // Check value
  EXPECT_DOUBLES_EQUAL(expected_error, f.error(values), 1e-9);

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

