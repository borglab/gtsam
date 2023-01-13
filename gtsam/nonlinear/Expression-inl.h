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

#include <gtsam/nonlinear/internal/ExpressionNode.h>

#include <boost/bind/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm.hpp>

namespace gtsam {

template<typename T>
Expression<T>::Expression(const T& value) :
    root_(new internal::ConstantExpression<T>(value)) {
}

template<typename T>
Expression<T>::Expression(const Key& key) :
    root_(new internal::LeafExpression<T>(key)) {
}

template<typename T>
Expression<T>::Expression(const Symbol& symbol) :
    root_(new internal::LeafExpression<T>(symbol)) {
}

template<typename T>
Expression<T>::Expression(unsigned char c, std::uint64_t j) :
    root_(new internal::LeafExpression<T>(Symbol(c, j))) {
}

/// Construct a unary function expression
template<typename T>
template<typename A>
Expression<T>::Expression(typename UnaryFunction<A>::type function,
    const Expression<A>& expression) :
    root_(new internal::UnaryExpression<T, A>(function, expression)) {
}

/// Construct a binary function expression
template<typename T>
template<typename A1, typename A2>
Expression<T>::Expression(typename BinaryFunction<A1, A2>::type function,
    const Expression<A1>& expression1, const Expression<A2>& expression2) :
    root_(
        new internal::BinaryExpression<T, A1, A2>(function, expression1,
            expression2)) {
}

/// Construct a ternary function expression
template<typename T>
template<typename A1, typename A2, typename A3>
Expression<T>::Expression(typename TernaryFunction<A1, A2, A3>::type function,
    const Expression<A1>& expression1, const Expression<A2>& expression2,
    const Expression<A3>& expression3) :
    root_(
        new internal::TernaryExpression<T, A1, A2, A3>(function, expression1,
            expression2, expression3)) {
}

/// Construct a nullary method expression
template<typename T>
template<typename A>
Expression<T>::Expression(const Expression<A>& expression,
    T (A::*method)(typename MakeOptionalJacobian<T, A>::type) const) :
    root_(
        new internal::UnaryExpression<T, A>(std::bind(method,
                std::placeholders::_1, std::placeholders::_2),
            expression)) {
}

/// Construct a unary method expression
template<typename T>
template<typename A1, typename A2>
Expression<T>::Expression(const Expression<A1>& expression1,
    T (A1::*method)(const A2&, typename MakeOptionalJacobian<T, A1>::type,
        typename MakeOptionalJacobian<T, A2>::type) const,
    const Expression<A2>& expression2) :
    root_(
        new internal::BinaryExpression<T, A1, A2>(
            std::bind(method, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4),
            expression1, expression2)) {
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
        new internal::TernaryExpression<T, A1, A2, A3>(
            std::bind(method, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5,
                std::placeholders::_6),
            expression1, expression2, expression3)) {
}

template<typename T>
std::set<Key> Expression<T>::keys() const {
  return root_->keys();
}

template<typename T>
void Expression<T>::dims(std::map<Key, int>& map) const {
  root_->dims(map);
}

template<typename T>
void Expression<T>::print(const std::string& s) const {
  root_->print(s);
}

template<typename T>
T Expression<T>::value(const Values& values,
    std::vector<Matrix>* H) const {
  if (H) {
    // Call private version that returns derivatives in H
    KeyVector keys;
    FastVector<int> dims;
    boost::tie(keys, dims) = keysAndDims();
    return valueAndDerivatives(values, keys, dims, *H);
  } else
    // no derivatives needed, just return value
    return root_->value(values);
}

template<typename T>
const boost::shared_ptr<internal::ExpressionNode<T> >& Expression<T>::root() const {
  return root_;
}

template<typename T>
size_t Expression<T>::traceSize() const {
  return root_->traceSize();
}

// Private methods:

template<typename T>
T Expression<T>::valueAndDerivatives(const Values& values,
    const KeyVector& keys, const FastVector<int>& dims,
    std::vector<Matrix>& H) const {

  // H should be pre-allocated
  assert(H.size()==keys.size());

  // Pre-allocate and zero VerticalBlockMatrix
  static const int Dim = traits<T>::dimension;
  VerticalBlockMatrix Ab(dims, Dim);
  Ab.matrix().setZero();
  internal::JacobianMap jacobianMap(keys, Ab);

  // Call unsafe version
  T result = valueAndJacobianMap(values, jacobianMap);

  // Copy blocks into the vector of jacobians passed in
  for (DenseIndex i = 0; i < static_cast<DenseIndex>(keys.size()); i++)
    H[i] = Ab(i);

  return result;
}

template<typename T>
T Expression<T>::traceExecution(const Values& values,
    internal::ExecutionTrace<T>& trace, void* traceStorage) const {
  return root_->traceExecution(values, trace,
      static_cast<internal::ExecutionTraceStorage*>(traceStorage));
}

template<typename T>
T Expression<T>::valueAndJacobianMap(const Values& values,
    internal::JacobianMap& jacobians) const {
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
  auto traceStorage = static_cast<internal::ExecutionTraceStorage*>(_aligned_malloc(size, internal::TraceAlignment));
#else
  internal::ExecutionTraceStorage traceStorage[size];
#endif

  internal::ExecutionTrace<T> trace;
  T value(this->traceExecution(values, trace, traceStorage));
  trace.startReverseAD1(jacobians);

#ifdef _MSC_VER
  _aligned_free(traceStorage);
#endif

  return value;
}

template<typename T>
typename Expression<T>::KeysAndDims Expression<T>::keysAndDims() const {
  std::map<Key, int> map;
  dims(map);
  size_t n = map.size();
  KeysAndDims pair = std::make_pair(KeyVector(n), FastVector<int>(n));
  boost::copy(map | boost::adaptors::map_keys, pair.first.begin());
  boost::copy(map | boost::adaptors::map_values, pair.second.begin());
  return pair;
}

namespace internal {
// http://stackoverflow.com/questions/16260445/boost-bind-to-operator
template<class T>
struct apply_compose {
  typedef T result_type;
  static const int Dim = traits<T>::dimension;
  T operator()(const T& x, const T& y, OptionalJacobian<Dim, Dim> H1 =
      {}, OptionalJacobian<Dim, Dim> H2 = {}) const {
    return x.compose(y, H1, H2);
  }
};

template <>
struct apply_compose<double> {
  double operator()(const double& x, const double& y,
                    OptionalJacobian<1, 1> H1 = {},
                    OptionalJacobian<1, 1> H2 = {}) const {
    if (H1) H1->setConstant(y);
    if (H2) H2->setConstant(x);
    return x * y;
  }
};

}

// Global methods:

/// Construct a product expression, assumes T::compose(T) -> T
template<typename T>
Expression<T> operator*(const Expression<T>& expression1,
    const Expression<T>& expression2) {
  return Expression<T>(
      std::bind(internal::apply_compose<T>(), std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4),
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

template <typename T>
ScalarMultiplyExpression<T>::ScalarMultiplyExpression(double s, const Expression<T>& e)
    : Expression<T>(boost::make_shared<internal::ScalarMultiplyNode<T>>(s, e)) {}


template <typename T>
BinarySumExpression<T>::BinarySumExpression(const Expression<T>& e1, const Expression<T>& e2)
    : Expression<T>(boost::make_shared<internal::BinarySumNode<T>>(e1, e2)) {}

template <typename T>
Expression<T>& Expression<T>::operator+=(const Expression<T>& e) {
  root_ = boost::make_shared<internal::BinarySumNode<T>>(*this, e);
  return *this;
}

} // namespace gtsam
