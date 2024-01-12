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

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>


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
    const auto [keys, dims] = keysAndDims();
    return valueAndDerivatives(values, keys, dims, *H);
  } else {
    // no derivatives needed, just return value
    return root_->value(values);
  }
}

template<typename T>
const std::shared_ptr<internal::ExpressionNode<T> >& Expression<T>::root() const {
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
    internal::ExecutionTrace<T>& trace, char* traceStorage) const {
  return root_->traceExecution(values, trace, traceStorage);
}

// Allocate a single block of aligned memory using a unique_ptr.
inline std::unique_ptr<internal::ExecutionTraceStorage[]> allocAligned(size_t size) {
  const size_t alignedSize = (size + internal::TraceAlignment - 1) / internal::TraceAlignment;
  return std::unique_ptr<internal::ExecutionTraceStorage[]>(
      new internal::ExecutionTraceStorage[alignedSize]);
}

template<typename T>
T Expression<T>::valueAndJacobianMap(const Values& values,
    internal::JacobianMap& jacobians) const {
  try {
    // We allocate a single block of aligned memory using a unique_ptr.
    const size_t size = traceSize();
    auto traceStorage = allocAligned(size);

    // The traceExecution call then fills this memory
    // with an execution trace, made up entirely of "Record" structs, see
    // the FunctionalNode class in expression-inl.h
    internal::ExecutionTrace<T> trace;
    T value(this->traceExecution(values, trace, reinterpret_cast<char *>(traceStorage.get())));

    // We then calculate the Jacobians using reverse automatic differentiation (AD).
    trace.startReverseAD1(jacobians);
    return value;
  } catch (const std::bad_alloc &e) {
    std::cerr << "valueAndJacobianMap exception: " << e.what() << '\n';
    throw e;
  }
  // Here traceStorage will be de-allocated properly.
}

template<typename T>
typename Expression<T>::KeysAndDims Expression<T>::keysAndDims() const {
  std::map<Key, int> map;
  dims(map);
  size_t n = map.size();
  KeysAndDims pair = {KeyVector(n), FastVector<int>(n)};
  // Copy map into pair of vectors
  auto key_it = pair.first.begin();
  auto dim_it = pair.second.begin();
  for (const auto& [key, value] : map) {
    *key_it++ = key;
    *dim_it++ = value;
  }
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

} // namespace internal

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
    : Expression<T>(std::make_shared<internal::ScalarMultiplyNode<T>>(s, e)) {}


template <typename T>
BinarySumExpression<T>::BinarySumExpression(const Expression<T>& e1, const Expression<T>& e2)
    : Expression<T>(std::make_shared<internal::BinarySumNode<T>>(e1, e2)) {}

template <typename T>
Expression<T>& Expression<T>::operator+=(const Expression<T>& e) {
  root_ = std::make_shared<internal::BinarySumNode<T>>(*this, e);
  return *this;
}

} // namespace gtsam
