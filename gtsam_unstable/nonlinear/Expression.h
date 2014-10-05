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
      root_(new LeafExpression<T>(Symbol(c, j))) {
  }

  /// Construct a nullary method expression
  template<typename A>
  Expression(const Expression<A>& expression,
      T (A::*method)(boost::optional<Matrix&>) const) {
    root_.reset(
        new UnaryExpression<T, A>(boost::bind(method, _1, _2), expression));
  }

  /// Construct a unary function expression
  template<typename A>
  Expression(typename UnaryExpression<T, A>::Function function,
      const Expression<A>& expression) {
    root_.reset(new UnaryExpression<T, A>(function, expression));
  }

  /// Construct a unary method expression
  template<typename A1, typename A2>
  Expression(const Expression<A1>& expression1,
      T (A1::*method)(const A2&, boost::optional<Matrix&>,
          boost::optional<Matrix&>) const, const Expression<A2>& expression2) {
    root_.reset(
        new BinaryExpression<T, A1, A2>(boost::bind(method, _1, _2, _3, _4),
            expression1, expression2));
  }

  /// Construct a binary function expression
  template<typename A1, typename A2>
  Expression(typename BinaryExpression<T, A1, A2>::Function function,
      const Expression<A1>& expression1, const Expression<A2>& expression2) {
    root_.reset(
        new BinaryExpression<T, A1, A2>(function, expression1, expression2));
  }

  /// Construct a ternary function expression
  template<typename A1, typename A2, typename A3>
  Expression(typename TernaryExpression<T, A1, A2, A3>::Function function,
      const Expression<A1>& expression1, const Expression<A2>& expression2,  const Expression<A3>& expression3) {
    root_.reset(
        new TernaryExpression<T, A1, A2, A3>(function, expression1, expression2, expression3));
  }

  /// Return keys that play in this expression
  std::set<Key> keys() const {
    return root_->keys();
  }

  /// Return value and optional derivatives
  T value(const Values& values) const {
    return root_->value(values);
  }

  /// Return value and derivatives
  Augmented<T> augmented(const Values& values) const {
    return root_->augmented(values);
  }

  const boost::shared_ptr<ExpressionNode<T> >& root() const {
    return root_;
  }

};

// http://stackoverflow.com/questions/16260445/boost-bind-to-operator
template<class T>
struct apply_compose {
  typedef T result_type;
  T operator()(const T& x, const T& y, boost::optional<Matrix&> H1,
      boost::optional<Matrix&> H2) const {
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

}

