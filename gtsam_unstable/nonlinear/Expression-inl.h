/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Expression-inl.h
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief Internals for Expression.h, not for general consumption
 */

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <boost/foreach.hpp>

namespace gtsam {

template<typename T>
class Expression;

/**
 * Expression node. The superclass for objects that do the heavy lifting
 * An Expression<T> has a pointer to an ExpressionNode<T> underneath
 * allowing Expressions to have polymorphic behaviour even though they
 * are passed by value. This is the same way boost::function works.
 * http://loki-lib.sourceforge.net/html/a00652.html
 */
template<class T>
class ExpressionNode {

protected:
  ExpressionNode() {
  }

public:

  typedef std::map<Key, Matrix> JacobianMap;

  /// Destructor
  virtual ~ExpressionNode() {
  }

  /// Return keys that play in this expression as a set
  virtual std::set<Key> keys() const = 0;

  /// Return value and optional derivatives
  virtual T value(const Values& values,
      boost::optional<JacobianMap&> = boost::none) const = 0;
};

/// Constant Expression
template<class T>
class ConstantExpression: public ExpressionNode<T> {

  T value_;

  /// Constructor with a value, yielding a constant
  ConstantExpression(const T& value) :
      value_(value) {
  }

  friend class Expression<T> ;

public:

  typedef std::map<Key, Matrix> JacobianMap;

  /// Destructor
  virtual ~ConstantExpression() {
  }

  /// Return keys that play in this expression, i.e., the empty set
  virtual std::set<Key> keys() const {
    std::set<Key> keys;
    return keys;
  }

  /// Return value and optional derivatives
  virtual T value(const Values& values,
      boost::optional<JacobianMap&> jacobians = boost::none) const {
    return value_;
  }
};

//-----------------------------------------------------------------------------
/// Leaf Expression
template<class T>
class LeafExpression: public ExpressionNode<T> {

  Key key_;

  /// Constructor with a single key
  LeafExpression(Key key) :
      key_(key) {
  }

  friend class Expression<T> ;

public:

  typedef std::map<Key, Matrix> JacobianMap;

  /// Destructor
  virtual ~LeafExpression() {
  }

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    std::set<Key> keys;
    keys.insert(key_);
    return keys;
  }

  /// Return value and optional derivatives
  virtual T value(const Values& values,
      boost::optional<JacobianMap&> jacobians = boost::none) const {
    const T& value = values.at<T>(key_);
    if (jacobians) {
      JacobianMap::iterator it = jacobians->find(key_);
      if (it != jacobians->end()) {
        it->second += Eigen::MatrixXd::Identity(value.dim(), value.dim());
      } else {
        (*jacobians)[key_] = Eigen::MatrixXd::Identity(value.dim(),
            value.dim());
      }
    }
    return value;
  }

};

//-----------------------------------------------------------------------------
/// Unary Expression
template<class T, class E>
class UnaryExpression: public ExpressionNode<T> {

public:

  typedef boost::function<T(const E&, boost::optional<Matrix&>)> function;

private:

  boost::shared_ptr<ExpressionNode<E> > expression_;
  function f_;

  /// Constructor with a unary function f, and input argument e
  UnaryExpression(function f, const Expression<E>& e) :
      expression_(e.root()), f_(f) {
  }

  friend class Expression<T> ;

public:

  typedef std::map<Key, Matrix> JacobianMap;

  /// Destructor
  virtual ~UnaryExpression() {
  }

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    return expression_->keys();
  }

  /// Return value and optional derivatives
  virtual T value(const Values& values,
      boost::optional<JacobianMap&> jacobians = boost::none) const {

    T value;
    if (jacobians) {
      Eigen::MatrixXd H;
      value = f_(expression_->value(values, jacobians), H);
      JacobianMap::iterator it = jacobians->begin();
      for (; it != jacobians->end(); ++it) {
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
class BinaryExpression: public ExpressionNode<T> {

public:

  typedef std::map<Key, Matrix> JacobianMap;

  typedef boost::function<
      T(const E1&, const E2&, boost::optional<Matrix&>,
          boost::optional<Matrix&>)> function;
private:

  boost::shared_ptr<ExpressionNode<E1> > expression1_;
  boost::shared_ptr<ExpressionNode<E2> > expression2_;
  function f_;

  /// Constructor with a binary function f, and two input arguments
  BinaryExpression(function f, //
      const Expression<E1>& e1, const Expression<E2>& e2) :
      expression1_(e1.root()), expression2_(e2.root()), f_(f) {
  }

  friend class Expression<T> ;

  /// Combine Jacobians
  static void combine(const Matrix& H1, const Matrix& H2,
      const JacobianMap& terms1, const JacobianMap& terms2,
      JacobianMap& jacobians) {
    // TODO: both Jacobians and terms are sorted. There must be a simple
    //       but fast algorithm that does this.
    typedef std::pair<Key, Matrix> Pair;
    BOOST_FOREACH(const Pair& term, terms1) {
      JacobianMap::iterator it = jacobians.find(term.first);
      if (it != jacobians.end()) {
        it->second += H1 * term.second;
      } else {
        jacobians[term.first] = H1 * term.second;
      }
    }
    BOOST_FOREACH(const Pair& term, terms2) {
      JacobianMap::iterator it = jacobians.find(term.first);
      if (it != jacobians.end()) {
        it->second += H2 * term.second;
      } else {
        jacobians[term.first] = H2 * term.second;
      }
    }
  }

public:

  /// Destructor
  virtual ~BinaryExpression() {
  }

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    std::set<Key> keys1 = expression1_->keys();
    std::set<Key> keys2 = expression2_->keys();
    keys1.insert(keys2.begin(), keys2.end());
    return keys1;
  }

  /// Return value and optional derivatives
  virtual T value(const Values& values,
      boost::optional<JacobianMap&> jacobians = boost::none) const {
    T val;
    if (jacobians) {
      JacobianMap terms1, terms2;
      Matrix H1, H2;
      val = f_(expression1_->value(values, terms1),
          expression2_->value(values, terms2), H1, H2);
      combine(H1, H2, terms1, terms2, *jacobians);
    } else {
      val = f_(expression1_->value(values), expression2_->value(values),
          boost::none, boost::none);
    }
    return val;
  }

};

}

