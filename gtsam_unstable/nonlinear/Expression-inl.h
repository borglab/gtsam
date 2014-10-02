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
#include <gtsam/base/Matrix.h>
#include <boost/foreach.hpp>

namespace gtsam {

template<typename T>
class Expression;
template<typename T, typename E1, typename E2>
class MethodExpression;

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
  virtual T value(const Values& values, boost::optional<JacobianMap&> =
      boost::none) const = 0;

protected:

  typedef std::pair<Key, Matrix> Pair;

  /// Insert terms into Jacobians, premultiplying by H, adding if already exists
  static void add(const Matrix& H, const JacobianMap& terms,
      JacobianMap& jacobians) {
    BOOST_FOREACH(const Pair& term, terms) {
      JacobianMap::iterator it = jacobians.find(term.first);
      if (it != jacobians.end()) {
        it->second += H * term.second;
      } else {
        jacobians[term.first] = H * term.second;
      }
    }
  }

  /// debugging
  static void print(const JacobianMap& terms, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) {
    BOOST_FOREACH(const Pair& term, terms) {
      std::cout << "(" << keyFormatter(term.first) << ", " << term.second.rows()
          << "x" << term.second.cols() << ") ";
    }
    std::cout << std::endl;
  }
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
    size_t n = value.dim();
    if (jacobians) {
      JacobianMap::iterator it = jacobians->find(key_);
      if (it != jacobians->end()) {
        it->second += Eigen::MatrixXd::Identity(n, n);
      } else {
        (*jacobians)[key_] = Eigen::MatrixXd::Identity(n, n);
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
      E1 arg1 = expression1_->value(values, terms1);
      E2 arg2 = expression2_->value(values, terms2);
      Matrix H1, H2;
      val = f_(arg1, arg2,
          terms1.empty() ? boost::none : boost::optional<Matrix&>(H1),
          terms2.empty() ? boost::none : boost::optional<Matrix&>(H2));
      ExpressionNode<T>::add(H1, terms1, *jacobians);
      ExpressionNode<T>::add(H2, terms2, *jacobians);
    } else {
      val = f_(expression1_->value(values), expression2_->value(values),
          boost::none, boost::none);
    }
    return val;
  }

};

//-----------------------------------------------------------------------------
/// Binary Expression

template<class T, class E1, class E2>
class MethodExpression: public ExpressionNode<T> {

public:

  typedef std::map<Key, Matrix> JacobianMap;

  typedef T (E1::*method)(const E2&, boost::optional<Matrix&>,
      boost::optional<Matrix&>) const;

private:

  boost::shared_ptr<ExpressionNode<E1> > expression1_;
  boost::shared_ptr<ExpressionNode<E2> > expression2_;
  method f_;

  /// Constructor with a binary function f, and two input arguments
  MethodExpression(const Expression<E1>& e1, method f, const Expression<E2>& e2) :
      expression1_(e1.root()), expression2_(e2.root()), f_(f) {
  }

  friend class Expression<T> ;

public:

  /// Destructor
  virtual ~MethodExpression() {
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
      E1 arg1 = expression1_->value(values, terms1);
      E2 arg2 = expression2_->value(values, terms2);
      Matrix H1, H2;
      val = (arg1.*(f_))(arg2,
          terms1.empty() ? boost::none : boost::optional<Matrix&>(H1),
          terms2.empty() ? boost::none : boost::optional<Matrix&>(H2));
      ExpressionNode<T>::add(H1, terms1, *jacobians);
      ExpressionNode<T>::add(H2, terms2, *jacobians);
    } else {
      val = (expression1_->value(values).*(f_))(expression2_->value(values),
          boost::none, boost::none);
    }
    return val;
  }

};

}

