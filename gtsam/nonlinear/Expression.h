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

#include <gtsam/nonlinear/Expression-inl.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/FastVector.h>

#include <boost/bind.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm.hpp>

class ExpressionFactorShallowTest;

namespace gtsam {

// Forward declare
template<typename T> class ExpressionFactor;

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
      T (A::*method)(typename MakeOptionalJacobian<T, A>::type) const) :
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
      T (A1::*method)(const A2&, typename MakeOptionalJacobian<T, A1>::type,
          typename MakeOptionalJacobian<T, A2>::type) const,
      const Expression<A2>& expression2) :
      root_(
          new BinaryExpression<T, A1, A2>(boost::bind(method, _1, _2, _3, _4),
              expression1, expression2)) {
  }

  /// Construct a binary function expression
  template<typename A1, typename A2>
  Expression(typename BinaryExpression<T, A1, A2>::Function function,
      const Expression<A1>& expression1, const Expression<A2>& expression2) :
      root_(new BinaryExpression<T, A1, A2>(function, expression1, expression2)) {
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

  /// Return root
  const boost::shared_ptr<ExpressionNode<T> >& root() const {
    return root_;
  }

  // Return size needed for memory buffer in traceExecution
  size_t traceSize() const {
    return root_->traceSize();
  }

  /// Return keys that play in this expression
  std::set<Key> keys() const {
    return root_->keys();
  }

  /// Return dimensions for each argument, as a map
  void dims(std::map<Key, int>& map) const {
    root_->dims(map);
  }

  /**
   * @brief Return value and optional derivatives, reverse AD version
   * Notes: this is not terribly efficient, and H should have correct size.
   * The order of the Jacobians is same as keys in either keys() or dims()
   */
  T value(const Values& values, boost::optional<std::vector<Matrix>&> H =
      boost::none) const {

    if (H) {
      // Call private version that returns derivatives in H
      KeysAndDims pair = keysAndDims();
      return value(values, pair.first, pair.second, *H);
    } else
      // no derivatives needed, just return value
      return root_->value(values);
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
      const FastVector<int>& dims, std::vector<Matrix>& H) const {

    // H should be pre-allocated
    assert(H.size()==keys.size());

    // Pre-allocate and zero VerticalBlockMatrix
    static const int Dim = traits::dimension<T>::value;
    VerticalBlockMatrix Ab(dims, Dim);
    Ab.matrix().setZero();
    JacobianMap jacobianMap(keys, Ab);

    // Call unsafe version
    T result = value(values, jacobianMap);

    // Copy blocks into the vector of jacobians passed in
    for (DenseIndex i = 0; i < static_cast<DenseIndex>(keys.size()); i++)
      H[i] = Ab(i);

    return result;
  }

  /// trace execution, very unsafe
  T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      ExecutionTraceStorage* traceStorage) const {
    return root_->traceExecution(values, trace, traceStorage);
  }

  /**
   * @brief Return value and derivatives, reverse AD version
   * This very unsafe method needs a JacobianMap with correctly allocated
   * and initialized VerticalBlockMatrix, hence is declared private.
   */
  T value(const Values& values, JacobianMap& jacobians) const {
    // The following piece of code is absolutely crucial for performance.
    // We allocate a block of memory on the stack, which can be done at runtime
    // with modern C++ compilers. The traceExecution then fills this memory
    // with an execution trace, made up entirely of "Record" structs, see
    // the FunctionalNode class in expression-inl.h
    size_t size = traceSize();

    // Windows does not support variable length arrays, so memory must be dynamically
    // allocated on Visual Studio. For more information see the issue below
    // https://bitbucket.org/gtborg/gtsam/issue/178/vlas-unsupported-in-visual-studio
#ifdef _MSC_VER
    ExecutionTraceStorage* traceStorage = new ExecutionTraceStorage[size];
#else
    ExecutionTraceStorage traceStorage[size];
#endif

    ExecutionTrace<T> trace;
    T value(traceExecution(values, trace, traceStorage));
    trace.startReverseAD1(jacobians);

#ifdef _MSC_VER
    delete[] traceStorage;
#endif

    return value;
  }

  // be very selective on who can access these private methods:
  friend class ExpressionFactor<T> ;
  friend class ::ExpressionFactorShallowTest;

};

// http://stackoverflow.com/questions/16260445/boost-bind-to-operator
template<class T>
struct apply_compose {
  typedef T result_type;
  static const int Dim = traits::dimension<T>::value;
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

