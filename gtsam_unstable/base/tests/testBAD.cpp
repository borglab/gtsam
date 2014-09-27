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

#include <CppUnitLite/TestHarness.h>

namespace gtsam {

///-----------------------------------------------------------------------------
/// Expression node. The superclass for objects that do the heavy lifting
/// An Expression<T> has a pointer to an ExpressionNode<T> underneath
/// allowing Expressions to have polymorphic behaviour even though they
/// are passed by value. This is the same way boost::function works.
/// http://loki-lib.sourceforge.net/html/a00652.html
template<class T>
class ExpressionNode {
 public:
  ExpressionNode(){}
  virtual ~ExpressionNode(){}
  virtual void getKeys(std::set<Key>& keys) const = 0;
  virtual T value(const Values& values,
                  boost::optional<std::map<Key, Matrix>&> = boost::none) const = 0;
};

template<typename T>
class Expression;

/// Constant Expression
template<class T>
class ConstantExpression : public ExpressionNode<T> {

  T value_;

 public:

  typedef T type;

  /// Constructor with a value, yielding a constant
  ConstantExpression(const T& value) :
    value_(value) {
  }
  virtual ~ConstantExpression(){}

  virtual void getKeys(std::set<Key>& /* keys */) const {}
  virtual T value(const Values& values,
                  boost::optional<std::map<Key, Matrix>&> jacobians = boost::none) const {
    return value_;
  }
};

//-----------------------------------------------------------------------------
/// Leaf Expression
template<class T>
class LeafExpression : public ExpressionNode<T> {

  Key key_;

 public:

  typedef T type;

  /// Constructor with a single key
  LeafExpression(Key key) :
    key_(key) {
  }
  virtual ~LeafExpression(){}

  virtual void getKeys(std::set<Key>& keys) const { keys.insert(key_); }
  virtual T value(const Values& values,
                  boost::optional<std::map<Key, Matrix>&> jacobians = boost::none) const {
    const T& value = values.at<T>(key_);
    if( jacobians ) {
      std::map<Key, Matrix>::iterator it = jacobians->find(key_);
      if(it != jacobians->end()) {
        it->second += Eigen::MatrixXd::Identity(value.dim(), value.dim());
      } else {
        (*jacobians)[key_] = Eigen::MatrixXd::Identity(value.dim(), value.dim());
      }
    }
    return value;
  }

};

//-----------------------------------------------------------------------------
/// Unary Expression
template<class T, class E>
class UnaryExpression : public ExpressionNode<T> {

 public:

  typedef T (*function)(const E&, boost::optional<Matrix&>);

 private:

  boost::shared_ptr< ExpressionNode<E> > expression_;
  function f_;

 public:

  typedef T type;

  /// Constructor with a single key
  UnaryExpression(function f, const Expression<E>& expression) :
    expression_(expression.root()), f_(f) {
  }
  virtual ~UnaryExpression(){}

  virtual void getKeys(std::set<Key>& keys) const{ expression_->getKeys(keys); }
  virtual T value(const Values& values,
                  boost::optional<std::map<Key, Matrix>&> jacobians = boost::none) const {

    T value;
    if(jacobians) {
      Eigen::MatrixXd H;
      value = f_(expression_->value(values, jacobians), H);
      std::map<Key, Matrix>::iterator it = jacobians->begin();
      for( ; it != jacobians->end(); ++it) {
        it->second = H * it->second;
      }
    } else {
      value = f_(expression_->value(values), boost::none);
    }
    return value;
  }

};

//-----------------------------------------------------------------------------
/// Binary Expression

template<class T, class E1, class E2>
class BinaryExpression : public ExpressionNode<T> {

 public:

  typedef T (*function)(const E1&, const E2&,
      boost::optional<Matrix&>, boost::optional<Matrix&>);

 private:

  boost::shared_ptr< ExpressionNode<E1> > expression1_;
  boost::shared_ptr< ExpressionNode<E2> > expression2_;
  function f_;

 public:

  typedef T type;

  /// Constructor with a single key
  BinaryExpression(function f, const Expression<E1>& expression1, const Expression<E2>& expression2) :
    expression1_(expression1.root()), expression2_(expression2.root()), f_(f) {
  }
  virtual ~BinaryExpression(){}

  virtual void getKeys(std::set<Key>& keys) const{
    expression1_->getKeys(keys);
    expression2_->getKeys(keys);
  }
  virtual T value(const Values& values,
                  boost::optional<std::map<Key, Matrix>&> jacobians = boost::none) const {
    T val;
    if(jacobians) {
      std::map<Key, Matrix> terms1;
      std::map<Key, Matrix> terms2;
      Matrix H1, H2;
      val = f_(expression1_->value(values, terms1), expression2_->value(values, terms2), H1, H2);
      // TODO: both Jacobians and terms are sorted. There must be a simple
      //       but fast algorithm that does this.
      typedef std::pair<Key, Matrix> Pair;
      BOOST_FOREACH(const Pair& term, terms1) {
        std::map<Key, Matrix>::iterator it = jacobians->find(term.first);
        if(it != jacobians->end()) {
          it->second += H1 * term.second;
        } else {
          (*jacobians)[term.first] = H1 * term.second;
        }
      }
      BOOST_FOREACH(const Pair& term, terms2) {
        std::map<Key, Matrix>::iterator it = jacobians->find(term.first);
        if(it != jacobians->end()) {
          it->second += H2 * term.second;
        } else {
          (*jacobians)[term.first] = H2 * term.second;
        }
      }
    } else {
      val = f_(expression1_->value(values), expression2_->value(values),
               boost::none, boost::none);
    }
    return val;
  }

};

template<typename T>
class Expression {
 public:

  // Initialize a constant expression
  Expression(const T& value) :
    root_(new ConstantExpression<T>(value)){ }

  // Initialize a leaf expression
  Expression(const Key& key) :
    root_(new LeafExpression<T>(key)) {}

  /// Initialize a unary expression
  template<typename E>
  Expression(typename UnaryExpression<T,E>::function f,
             const Expression<E>& expression) {
    // TODO Assert that root of expression is not null.
    root_.reset(new UnaryExpression<T,E>(f, expression));
  }

  /// Initialize a binary expression
  template<typename E1, typename E2>
  Expression(typename BinaryExpression<T,E1,E2>::function f,
             const Expression<E1>& expression1,
             const Expression<E2>& expression2) {
    // TODO Assert that root of expressions 1 and 2 are not null.
    root_.reset(new BinaryExpression<T,E1,E2>(f, expression1,
                                              expression2));
  }

  void getKeys(std::set<Key>& keys) const { root_->getKeys(keys); }
  T value(const Values& values,
          boost::optional<std::map<Key, Matrix>&> jacobians = boost::none) const {
    return root_->value(values, jacobians);
  }

  const boost::shared_ptr<ExpressionNode<T> >& root() const{ return root_; }
 private:
  boost::shared_ptr<ExpressionNode<T> > root_;
};
//-----------------------------------------------------------------------------

void printPair(std::pair<Key, Matrix> pair) {
  std::cout << pair.first << ": " << pair.second << std::endl;
}
// usage: std::for_each(terms.begin(), terms.end(), printPair);

//-----------------------------------------------------------------------------
/// AD Factor
template<class T>
class BADFactor: NonlinearFactor {

  const T measurement_;
  const Expression<T> expression_;

  /// get value from expression and calculate error with respect to measurement
  Vector unwhitenedError(const Values& values) const {
    const T& value = expression_.value(values);
    return value.localCoordinates(measurement_);
  }

 public:

  /// Constructor
  BADFactor(const T& measurement, const Expression<T>& expression) :
    measurement_(measurement), expression_(expression) {
  }
  /// Constructor
  BADFactor(const T& measurement, const ExpressionNode<T>& expression) :
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
    std::map<Key, Matrix> terms;
    expression_.value(values, terms);
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

Point3 transformTo(const Pose3& x, const Point3& p,
                   boost::optional<Matrix&> Dpose, boost::optional<Matrix&> Dpoint) {
  return x.transform_to(p, Dpose, Dpoint);
}

Point2 project(const Point3& p, boost::optional<Matrix&> Dpoint) {
  return PinholeCamera<Cal3_S2>::project_to_camera(p, Dpoint);
}

template<class CAL>
Point2 uncalibrate(const CAL& K, const Point2& p, boost::optional<Matrix&> Dcal,
                   boost::optional<Matrix&> Dp) {
  return K.uncalibrate(p, Dcal, Dp);
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
  Expression<Pose3> x(1);
  Expression<Point3> p(2);
  Expression<Cal3_S2> K(3);

  // Create expression tree
  Expression<Point3> p_cam(transformTo, x, p);
  Expression<Point2> projection(project, p_cam);
  Expression<Point2> uv_hat(uncalibrate, K, projection);

  // Check getKeys
  std::set<Key> keys;
  uv_hat.getKeys(keys);
  EXPECT_LONGS_EQUAL(3, keys.size());

  // Create factor
  BADFactor<Point2> f(measured, uv_hat);

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

