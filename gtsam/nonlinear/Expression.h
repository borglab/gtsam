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

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/base/VerticalBlockMatrix.h>

#include <boost/bind.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/make_shared.hpp>

class ExpressionFactorShallowTest;

namespace gtsam {

// Forward declares
class Values;
template<typename T> class ExecutionTrace;
template<typename T> class ExpressionNode;
template<typename T> class ExpressionFactor;

// A JacobianMap is the primary mechanism by which derivatives are returned.
// For clarity, it is forward declared here but implemented at the end of this header.
class JacobianMap;

// Expressions wrap trees of functions that can evaluate their own derivatives.
// The meta-functions below provide a handy to specify the type of those functions
template<class T, class A1>
struct UnaryFunction {
  typedef boost::function<
      T(const A1&, typename MakeOptionalJacobian<T, A1>::type)> type;
};

template<class T, class A1, class A2>
struct BinaryFunction {
  typedef boost::function<
      T(const A1&, const A2&, typename MakeOptionalJacobian<T, A1>::type,
          typename MakeOptionalJacobian<T, A2>::type)> type;
};

template<class T, class A1, class A2, class A3>
struct TernaryFunction {
  typedef boost::function<
      T(const A1&, const A2&, const A3&,
          typename MakeOptionalJacobian<T, A1>::type,
          typename MakeOptionalJacobian<T, A2>::type,
          typename MakeOptionalJacobian<T, A3>::type)> type;
};

/// Storage type for the execution trace.
/// It enforces the proper alignment in a portable way.
/// Provide a traceSize() sized array of this type to traceExecution as traceStorage.
const unsigned TraceAlignment = 16;
typedef boost::aligned_storage<1, TraceAlignment>::type ExecutionTraceStorage;

/**
 * Expression class that supports automatic differentiation
 */
template<typename T>
class Expression {

public:

  /// Define type so we can apply it as a meta-function
  typedef Expression<T> type;

private:

  // Paul's trick shared pointer, polymorphic root of entire expression tree
  boost::shared_ptr<ExpressionNode<T> > root_;

public:

  /// Print
  void print(const std::string& s) const;

  /// Construct a constant expression
  Expression(const T& value);

  /// Construct a leaf expression, with Key
  Expression(const Key& key);

  /// Construct a leaf expression, with Symbol
  Expression(const Symbol& symbol);

  /// Construct a leaf expression, creating Symbol
  Expression(unsigned char c, size_t j);

  /// Construct a nullary method expression
  template<typename A>
  Expression(const Expression<A>& expression,
      T (A::*method)(typename MakeOptionalJacobian<T, A>::type) const);

  /// Construct a unary function expression
  template<typename A>
  Expression(typename UnaryFunction<T, A>::type function,
      const Expression<A>& expression);

  /// Construct a unary method expression
  template<typename A1, typename A2>
  Expression(const Expression<A1>& expression1,
      T (A1::*method)(const A2&, typename MakeOptionalJacobian<T, A1>::type,
          typename MakeOptionalJacobian<T, A2>::type) const,
      const Expression<A2>& expression2);

  /// Construct a binary function expression
  template<typename A1, typename A2>
  Expression(typename BinaryFunction<T, A1, A2>::type function,
      const Expression<A1>& expression1, const Expression<A2>& expression2);

  /// Construct a binary method expression
  template<typename A1, typename A2, typename A3>
  Expression(const Expression<A1>& expression1,
      T (A1::*method)(const A2&, const A3&,
          typename MakeOptionalJacobian<T, A1>::type,
          typename MakeOptionalJacobian<T, A2>::type,
          typename MakeOptionalJacobian<T, A3>::type) const,
      const Expression<A2>& expression2, const Expression<A3>& expression3);

  /// Construct a ternary function expression
  template<typename A1, typename A2, typename A3>
  Expression(typename TernaryFunction<T, A1, A2, A3>::type function,
      const Expression<A1>& expression1, const Expression<A2>& expression2,
      const Expression<A3>& expression3);

  /// Return root
  const boost::shared_ptr<ExpressionNode<T> >& root() const;

  // Return size needed for memory buffer in traceExecution
  size_t traceSize() const;

  /// Return keys that play in this expression
  std::set<Key> keys() const;

  /// Return dimensions for each argument, as a map
  void dims(std::map<Key, int>& map) const;

  /**
   * @brief Return value and optional derivatives, reverse AD version
   * Notes: this is not terribly efficient, and H should have correct size.
   * The order of the Jacobians is same as keys in either keys() or dims()
   */
  T value(const Values& values, boost::optional<std::vector<Matrix>&> H =
      boost::none) const;

  /**
   *  @return a "deep" copy of this Expression
   *  "deep" is in quotes because the ExpressionNode hierarchy is *not* cloned.
   *  The intent is for derived classes to be copied using only a Base pointer.
   */
  virtual boost::shared_ptr<Expression> clone() const {
    return boost::make_shared<Expression>(*this);
  }

private:

  /// Vaguely unsafe keys and dimensions in same order
  typedef std::pair<FastVector<Key>, FastVector<int> > KeysAndDims;
  KeysAndDims keysAndDims() const {
    std::map<Key, int> map;
    dims(map);
    size_t n = map.size();
    KeysAndDims pair = std::make_pair(FastVector<Key>(n), FastVector<int>(n));
    boost::copy(map | boost::adaptors::map_keys, pair.first.begin());
    boost::copy(map | boost::adaptors::map_values, pair.second.begin());
    return pair;
  }

  /// private version that takes keys and dimensions, returns derivatives
  T value(const Values& values, const FastVector<Key>& keys,
      const FastVector<int>& dims, std::vector<Matrix>& H) const;

  /// trace execution, very unsafe
  T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      ExecutionTraceStorage* traceStorage) const;

  /**
   * @brief Return value and derivatives, reverse AD version
   * This very unsafe method needs a JacobianMap with correctly allocated
   * and initialized VerticalBlockMatrix, hence is declared private.
   */
  T value(const Values& values, JacobianMap& jacobians) const;

  // be very selective on who can access these private methods:
  friend class ExpressionFactor<T> ;
  friend class ::ExpressionFactorShallowTest;

};

// Expressions are designed to write their derivatives into an already allocated
// Jacobian of the correct size, of type VerticalBlockMatrix.
// The JacobianMap provides a mapping from keys to the underlying blocks.
class JacobianMap {
private:
  const FastVector<Key>& keys_;
  VerticalBlockMatrix& Ab_;

public:
  /// Construct a JacobianMap for writing into a VerticalBlockMatrix Ab
  JacobianMap(const FastVector<Key>& keys, VerticalBlockMatrix& Ab);

  /// Access blocks of via key
  VerticalBlockMatrix::Block operator()(Key key);
};

// http://stackoverflow.com/questions/16260445/boost-bind-to-operator
template<class T>
struct apply_compose {
  typedef T result_type;
  static const int Dim = traits<T>::dimension;
  T operator()(const T& x, const T& y, OptionalJacobian<Dim, Dim> H1 =
      boost::none, OptionalJacobian<Dim, Dim> H2 = boost::none) const {
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

#include <gtsam/nonlinear/Expression-inl.h>

