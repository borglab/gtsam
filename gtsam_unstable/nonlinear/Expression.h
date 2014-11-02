/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Expression.h
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief Expressions for Block Automatic Differentiation
 */

#pragma once

#include "Expression-inl.h"
#include <gtsam/inference/Symbol.h>
#include <boost/bind.hpp>

namespace gtsam {

/**
 * Expression class that supports automatic differentiation
 */
template<typename T>
class Expression {

private:

  // Paul's trick shared pointer, polymorphic root of entire expression tree
  boost::shared_ptr<ExpressionNode<T> > root_;

public:

  // Construct a constant expression
  Expression(const T& value) :
      root_(new ConstantExpression<T>(value)) {
  }

  // Construct a leaf expression, with Key
  Expression(const Key& key) :
      root_(new LeafExpression<T>(key)) {
  }

  // Construct a leaf expression, with Symbol
  Expression(const Symbol& symbol) :
      root_(new LeafExpression<T>(symbol)) {
  }

  // Construct a leaf expression, creating Symbol
  Expression(unsigned char c, size_t j) :
      root_(new LeafExpression<T>(Symbol(c, j)))  {
  }

  /// Construct a nullary method expression
  template<typename A>
  Expression(const Expression<A>& expression,
      T (A::*method)(typename OptionalJacobian<T, A>::type) const) :
      root_(new UnaryExpression<T, A>(boost::bind(method, _1, _2), expression)) {
  }

  /// Construct a unary function expression
  template<typename A>
  Expression(typename UnaryExpression<T, A>::Function function,
      const Expression<A>& expression) :
      root_(new UnaryExpression<T, A>(function, expression)) {
  }

  /// Construct a unary method expression
  template<typename A1, typename A2>
  Expression(const Expression<A1>& expression1,
      T (A1::*method)(const A2&, typename OptionalJacobian<T, A1>::type,
          typename OptionalJacobian<T, A2>::type) const,
      const Expression<A2>& expression2) :
      root_(
          new BinaryExpression<T, A1, A2>(boost::bind(method, _1, _2, _3, _4),
              expression1, expression2)) {
  }

  /// Construct a binary function expression
  template<typename A1, typename A2>
  Expression(typename BinaryExpression<T, A1, A2>::Function function,
      const Expression<A1>& expression1, const Expression<A2>& expression2) :
      root_(
          new BinaryExpression<T, A1, A2>(function, expression1, expression2)) {
  }

  /// Construct a ternary function expression
  template<typename A1, typename A2, typename A3>
  Expression(typename TernaryExpression<T, A1, A2, A3>::Function function,
      const Expression<A1>& expression1, const Expression<A2>& expression2,
      const Expression<A3>& expression3) :
      root_(
          new TernaryExpression<T, A1, A2, A3>(function, expression1,
              expression2, expression3)) {
  }

  /// Return keys that play in this expression
  std::set<Key> keys() const {
    return root_->keys();
  }

  /// Return dimensions for each argument, as a map
  void dims(std::map<Key, size_t>& map) const {
    root_->dims(map);
  }

  // Return size needed for memory buffer in traceExecution
  size_t traceSize() const {
    return root_->traceSize();
  }

  /// trace execution, very unsafe, for testing purposes only
  T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const {
    return root_->traceExecution(values, trace, raw);
  }

  /// Return value and derivatives, reverse AD version
  T value(const Values& values, JacobianMap& jacobians) const {
    // The following piece of code is absolutely crucial for performance.
    // We allocate a block of memory on the stack, which can be done at runtime
    // with modern C++ compilers. The traceExecution then fills this memory
    // with an execution trace, made up entirely of "Record" structs, see
    // the FunctionalNode class in expression-inl.h
    size_t size = traceSize();
    char raw[size];
    ExecutionTrace<T> trace;
    T value(traceExecution(values, trace, raw));
    trace.startReverseAD(jacobians);
    return value;
  }

  /// Return value
  T value(const Values& values) const {
    return root_->value(values);
  }

  const boost::shared_ptr<ExpressionNode<T> >& root() const {
    return root_;
  }

  /// Define type so we can apply it as a meta-function
  typedef Expression<T> type;
};

// http://stackoverflow.com/questions/16260445/boost-bind-to-operator
template<class T>
struct apply_compose {
  typedef T result_type;
  static const int Dim = traits::dimension<T>::value;
  typedef Eigen::Matrix<double, Dim, Dim> Jacobian;
  T operator()(const T& x, const T& y, boost::optional<Jacobian&> H1,
      boost::optional<Jacobian&> H2) const {
    return x.compose(y, H1, H2);
  }
};

/// Construct a product expression, assumes T::compose(T) -> T
template<typename T>
Expression<T> operator*(const Expression<T>& expression1,
    const Expression<T>& expression2) {
  return Expression<T>(boost::bind(apply_compose<T>(), _1, _2, _3, _4),
      expression1, expression2);
}

/// Construct an array of leaves
template<typename T>
std::vector<Expression<T> > createUnknowns(size_t n, char c, size_t start = 0) {
  std::vector<Expression<T> > unknowns;
  unknowns.reserve(n);
  for (size_t i = start; i < start + n; i++)
    unknowns.push_back(Expression<T>(c, i));
  return unknowns;
}

}

