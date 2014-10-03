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

typedef std::map<Key, Matrix> JacobianMap;

//-----------------------------------------------------------------------------
/**
 * Value and Jacobians
 */
template<class T>
class Augmented {

private:

  T value_;
  JacobianMap jacobians_;

  typedef std::pair<Key, Matrix> Pair;

  /// Insert terms into jacobians_, premultiplying by H, adding if already exists
  void add(const Matrix& H, const JacobianMap& terms) {
    BOOST_FOREACH(const Pair& term, terms) {
      JacobianMap::iterator it = jacobians_.find(term.first);
      if (it != jacobians_.end())
        it->second += H * term.second;
      else
        jacobians_[term.first] = H * term.second;
    }
  }

public:

  /// Construct value that does not depend on anything
  Augmented(const T& t) :
      value_(t) {
  }

  /// Construct value dependent on a single key
  Augmented(const T& t, Key key) :
      value_(t) {
    size_t n = t.dim();
    jacobians_[key] = Eigen::MatrixXd::Identity(n, n);
  }

  /// Construct value, pre-multiply jacobians by H
  Augmented(const T& t, const Matrix& H, const JacobianMap& jacobians) :
      value_(t) {
    add(H, jacobians);
  }

  /// Construct value, pre-multiply jacobians by H
  Augmented(const T& t, const Matrix& H1, const JacobianMap& jacobians1,
      const Matrix& H2, const JacobianMap& jacobians2) :
      value_(t) {
    add(H1, jacobians1);
    add(H2, jacobians2);
  }

  /// Return value
  const T& value() const {
    return value_;
  }

  /// Return jacobians
  const JacobianMap& jacobians() const {
    return jacobians_;
  }

  /// Not dependent on any key
  bool constant() const {
    return jacobians_.empty();
  }

  /// debugging
  void print(const KeyFormatter& keyFormatter = DefaultKeyFormatter) {
    BOOST_FOREACH(const Pair& term, jacobians_)
      std::cout << "(" << keyFormatter(term.first) << ", " << term.second.rows()
          << "x" << term.second.cols() << ") ";
    std::cout << std::endl;
  }
};

//-----------------------------------------------------------------------------
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

  /// Destructor
  virtual ~ExpressionNode() {
  }

  /// Return keys that play in this expression as a set
  virtual std::set<Key> keys() const = 0;

  /// Return value
  virtual T value(const Values& values) const = 0;

  /// Return value and derivatives
  virtual Augmented<T> augmented(const Values& values) const = 0;

};

//-----------------------------------------------------------------------------
/// Constant Expression
template<class T>
class ConstantExpression: public ExpressionNode<T> {

  /// The constant value
  T constant_;

  /// Constructor with a value, yielding a constant
  ConstantExpression(const T& value) :
      constant_(value) {
  }

  friend class Expression<T> ;

public:

  /// Destructor
  virtual ~ConstantExpression() {
  }

  /// Return keys that play in this expression, i.e., the empty set
  virtual std::set<Key> keys() const {
    std::set<Key> keys;
    return keys;
  }

  /// Return value
  virtual T value(const Values& values) const {
    return constant_;
  }

  /// Return value and derivatives
  virtual Augmented<T> augmented(const Values& values) const {
    T t = value(values);
    return Augmented<T>(t);
  }

};

//-----------------------------------------------------------------------------
/// Leaf Expression
template<class T>
class LeafExpression: public ExpressionNode<T> {

  /// The key into values
  Key key_;

  /// Constructor with a single key
  LeafExpression(Key key) :
      key_(key) {
  }

  friend class Expression<T> ;

public:

  /// Destructor
  virtual ~LeafExpression() {
  }

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    std::set<Key> keys;
    keys.insert(key_);
    return keys;
  }

  /// Return value
  virtual T value(const Values& values) const {
    return values.at<T>(key_);
  }

  /// Return value and derivatives
  virtual Augmented<T> augmented(const Values& values) const {
    T t = value(values);
    return Augmented<T>(t, key_);
  }

};

//-----------------------------------------------------------------------------
/// Unary Expression
template<class T, class A>
class UnaryExpression: public ExpressionNode<T> {

protected:

  boost::shared_ptr<ExpressionNode<A> > expressionA_;

  /// Constructor with one input argument expression
  UnaryExpression(const Expression<A>& e) :
      expressionA_(e.root()) {
  }

public:

  /// Destructor
  virtual ~UnaryExpression() {
  }

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    return expressionA_->keys();
  }

};

//-----------------------------------------------------------------------------
/// Nullary Method Expression
template<class T, class A>
class NullaryMethodExpression: public UnaryExpression<T, A> {

public:

  typedef T (A::*method)(boost::optional<Matrix&>) const;

private:

  method method_;

  /// Constructor with a unary function f, and input argument e
  NullaryMethodExpression(const Expression<A>& e, method f) :
      UnaryExpression<T, A>(e), method_(f) {
  }

  friend class Expression<T> ;

public:

  /// Destructor
  virtual ~NullaryMethodExpression() {
  }

  /// Return value
  virtual T value(const Values& values) const {
    using boost::none;
    return (this->expressionA_->value(values).*(method_))(none);
  }

  /// Return value and derivatives
  virtual Augmented<T> augmented(const Values& values) const {
    using boost::none;
    Augmented<A> argument = this->expressionA_->augmented(values);
    Matrix H;
    T t = (argument.value().*(method_))(
        argument.constant() ? none : boost::optional<Matrix&>(H));
    return Augmented<T>(t, H, argument.jacobians());
  }

};

//-----------------------------------------------------------------------------
/// Unary Function Expression
template<class T, class A>
class UnaryFunctionExpression: public UnaryExpression<T, A> {

public:

  typedef boost::function<T(const A&, boost::optional<Matrix&>)> function;

private:

  function function_;

  /// Constructor with a unary function f, and input argument e
  UnaryFunctionExpression(function f, const Expression<A>& e) :
      UnaryExpression<T, A>(e), function_(f) {
  }

  friend class Expression<T> ;

public:

  /// Destructor
  virtual ~UnaryFunctionExpression() {
  }

  /// Return value
  virtual T value(const Values& values) const {
    return function_(this->expressionA_->value(values), boost::none);
  }

  /// Return value and derivatives
  virtual Augmented<T> augmented(const Values& values) const {
    using boost::none;
    Augmented<A> argument = this->expressionA_->augmented(values);
    Matrix H;
    T t = function_(argument.value(),
        argument.constant() ? none : boost::optional<Matrix&>(H));
    return Augmented<T>(t, H, argument.jacobians());
  }

};

//-----------------------------------------------------------------------------
/// Binary function Expression

template<class T, class A1, class A2>
class BinaryExpression: public ExpressionNode<T> {

public:

  typedef boost::function<
      T(const A1&, const A2&, boost::optional<Matrix&>,
          boost::optional<Matrix&>)> function;

private:

  boost::shared_ptr<ExpressionNode<A1> > expressionA1_;
  boost::shared_ptr<ExpressionNode<A2> > expressionA2_;
  function function_;

  /// Constructor with a binary function f, and two input arguments
  BinaryExpression(function f, //
      const Expression<A1>& e1, const Expression<A2>& e2) :
      expressionA1_(e1.root()), expressionA2_(e2.root()), function_(f) {
  }

  friend class Expression<T> ;

public:

  /// Destructor
  virtual ~BinaryExpression() {
  }

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    std::set<Key> keys1 = expressionA1_->keys();
    std::set<Key> keys2 = expressionA2_->keys();
    keys1.insert(keys2.begin(), keys2.end());
    return keys1;
  }

  /// Return value
  virtual T value(const Values& values) const {
    using boost::none;
    return function_(expressionA1_->value(values), expressionA2_->value(values),
        none, none);
  }

  /// Return value and derivatives
  virtual Augmented<T> augmented(const Values& values) const {
    using boost::none;
    Augmented<A1> argument1 = expressionA1_->augmented(values);
    Augmented<A2> argument2 = expressionA2_->augmented(values);
    Matrix H1, H2;
    T t = function_(argument1.value(), argument2.value(),
        argument1.constant() ? none : boost::optional<Matrix&>(H1),
        argument2.constant() ? none : boost::optional<Matrix&>(H2));
    return Augmented<T>(t, H1, argument1.jacobians(), H2, argument2.jacobians());
  }

};

//-----------------------------------------------------------------------------
/// Binary Expression

template<class T, class A1, class A2>
class MethodExpression: public ExpressionNode<T> {

public:

  typedef T (A1::*method)(const A2&, boost::optional<Matrix&>,
      boost::optional<Matrix&>) const;

private:

  boost::shared_ptr<ExpressionNode<A1> > expressionA1_;
  boost::shared_ptr<ExpressionNode<A2> > expressionA2_;
  method method_;

  /// Constructor with a binary function f, and two input arguments
  MethodExpression(const Expression<A1>& e1, method f, const Expression<A2>& e2) :
      expressionA1_(e1.root()), expressionA2_(e2.root()), method_(f) {
  }

  friend class Expression<T> ;

public:

  /// Destructor
  virtual ~MethodExpression() {
  }

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    std::set<Key> keys1 = expressionA1_->keys();
    std::set<Key> keys2 = expressionA2_->keys();
    keys1.insert(keys2.begin(), keys2.end());
    return keys1;
  }

  /// Return value
  virtual T value(const Values& values) const {
    using boost::none;
    return (expressionA1_->value(values).*(method_))(expressionA2_->value(values),
        none, none);
  }

  /// Return value and derivatives
  virtual Augmented<T> augmented(const Values& values) const {
    using boost::none;
    Augmented<A1> argument1 = expressionA1_->augmented(values);
    Augmented<A2> argument2 = expressionA2_->augmented(values);
    Matrix H1, H2;
    T t = (argument1.value().*(method_))(argument2.value(),
        argument1.constant() ? none : boost::optional<Matrix&>(H1),
        argument2.constant() ? none : boost::optional<Matrix&>(H2));
    return Augmented<T>(t, H1, argument1.jacobians(), H2, argument2.jacobians());
  }

};

}

