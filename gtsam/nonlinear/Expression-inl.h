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

#include <gtsam/nonlinear/ExpressionNode.h>

#include <boost/tuple/tuple.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm.hpp>

namespace gtsam {

/// Print
template<typename T>
void Expression<T>::print(const std::string& s) const {
  std::cout << s << *root_ << std::endl;
}

// Construct a constant expression
template<typename T>
Expression<T>::Expression(const T& value) :
    root_(new ConstantExpression<T>(value)) {
}

// Construct a leaf expression, with Key
template<typename T>
Expression<T>::Expression(const Key& key) :
    root_(new LeafExpression<T>(key)) {
}

// Construct a leaf expression, with Symbol
template<typename T>
Expression<T>::Expression(const Symbol& symbol) :
    root_(new LeafExpression<T>(symbol)) {
}

// Construct a leaf expression, creating Symbol
template<typename T>
Expression<T>::Expression(unsigned char c, size_t j) :
    root_(new LeafExpression<T>(Symbol(c, j))) {
}

/// Construct a nullary method expression
template<typename T>
template<typename A>
Expression<T>::Expression(const Expression<A>& expression,
    T (A::*method)(typename MakeOptionalJacobian<T, A>::type) const) :
    root_(new UnaryExpression<T, A>(boost::bind(method, _1, _2), expression)) {
}

/// Construct a unary function expression
template<typename T>
template<typename A>
Expression<T>::Expression(typename UnaryFunction<A>::type function,
    const Expression<A>& expression) :
    root_(new UnaryExpression<T, A>(function, expression)) {
}

/// Construct a unary method expression
template<typename T>
template<typename A1, typename A2>
Expression<T>::Expression(const Expression<A1>& expression1,
    T (A1::*method)(const A2&, typename MakeOptionalJacobian<T, A1>::type,
        typename MakeOptionalJacobian<T, A2>::type) const,
    const Expression<A2>& expression2) :
    root_(
        new BinaryExpression<T, A1, A2>(boost::bind(method, _1, _2, _3, _4),
            expression1, expression2)) {
}

/// Construct a binary function expression
template<typename T>
template<typename A1, typename A2>
Expression<T>::Expression(typename BinaryFunction<A1, A2>::type function,
    const Expression<A1>& expression1, const Expression<A2>& expression2) :
    root_(new BinaryExpression<T, A1, A2>(function, expression1, expression2)) {
}

/// Construct a binary method expression
template<typename T>
template<typename A1, typename A2, typename A3>
Expression<T>::Expression(const Expression<A1>& expression1,
    T (A1::*method)(const A2&, const A3&,
        typename MakeOptionalJacobian<T, A1>::type,
        typename MakeOptionalJacobian<T, A2>::type,
        typename MakeOptionalJacobian<T, A3>::type) const,
    const Expression<A2>& expression2, const Expression<A3>& expression3) :
    root_(
        new TernaryExpression<T, A1, A2, A3>(
            boost::bind(method, _1, _2, _3, _4, _5, _6), expression1,
            expression2, expression3)) {
}

/// Construct a ternary function expression
template<typename T>
template<typename A1, typename A2, typename A3>
Expression<T>::Expression(typename TernaryFunction<A1, A2, A3>::type function,
    const Expression<A1>& expression1, const Expression<A2>& expression2,
    const Expression<A3>& expression3) :
    root_(
        new TernaryExpression<T, A1, A2, A3>(function, expression1, expression2,
            expression3)) {
}

/// Return root
template<typename T>
const boost::shared_ptr<ExpressionNode<T> >& Expression<T>::root() const {
  return root_;
}

// Return size needed for memory buffer in traceExecution
template<typename T>
size_t Expression<T>::traceSize() const {
  return root_->traceSize();
}

/// Return keys that play in this expression
template<typename T>
std::set<Key> Expression<T>::keys() const {
  return root_->keys();
}

/// Return dimensions for each argument, as a map
template<typename T>
void Expression<T>::dims(std::map<Key, int>& map) const {
  root_->dims(map);
}

/**
 * @brief Return value and optional derivatives, reverse AD version
 * Notes: this is not terribly efficient, and H should have correct size.
 * The order of the Jacobians is same as keys in either keys() or dims()
 */
template<typename T>
T Expression<T>::value(const Values& values,
    boost::optional<std::vector<Matrix>&> H) const {

  if (H) {
    // Call private version that returns derivatives in H
    KeysAndDims pair = keysAndDims();
    return value(values, pair.first, pair.second, *H);
  } else
    // no derivatives needed, just return value
    return root_->value(values);
}

/// private version that takes keys and dimensions, returns derivatives
template<typename T>
T Expression<T>::value(const Values& values, const FastVector<Key>& keys,
    const FastVector<int>& dims, std::vector<Matrix>& H) const {

  // H should be pre-allocated
  assert(H.size()==keys.size());

  // Pre-allocate and zero VerticalBlockMatrix
  static const int Dim = traits<T>::dimension;
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

template<typename T>
T Expression<T>::traceExecution(const Values& values, ExecutionTrace<T>& trace,
    void* traceStorage) const {
  return root_->traceExecution(values, trace,
      static_cast<ExecutionTraceStorage*>(traceStorage));
}

template<typename T>
T Expression<T>::value(const Values& values, JacobianMap& jacobians) const {
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
  T value(this->traceExecution(values, trace, traceStorage));
  trace.startReverseAD1(jacobians);

#ifdef _MSC_VER
  delete[] traceStorage;
#endif

  return value;
}

template<typename T>
typename Expression<T>::KeysAndDims Expression<T>::keysAndDims() const {
  std::map<Key, int> map;
  dims(map);
  size_t n = map.size();
  KeysAndDims pair = std::make_pair(FastVector < Key > (n), FastVector<int>(n));
  boost::copy(map | boost::adaptors::map_keys, pair.first.begin());
  boost::copy(map | boost::adaptors::map_values, pair.second.begin());
  return pair;
}

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
std::vector<Expression<T> > createUnknowns(size_t n, char c, size_t start) {
  std::vector<Expression<T> > unknowns;
  unknowns.reserve(n);
  for (size_t i = start; i < start + n; i++)
    unknowns.push_back(Expression<T>(c, i));
  return unknowns;
}

} // namespace gtsam
