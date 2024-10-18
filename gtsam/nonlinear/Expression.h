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

#include <gtsam/nonlinear/internal/JacobianMap.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/VectorSpace.h>

#include <map>

// Forward declare tests
class ExpressionFactorShallowTest;

namespace gtsam {

// Forward declares
class Values;
template<typename T> class ExpressionFactor;
template<typename T> class ExpressionEqualityConstraint;
class ScalarExpressionInequalityConstraint;

namespace internal {
template<typename T> class ExecutionTrace;
template<typename T> class ExpressionNode;
}

/**
 * Expression class that supports automatic differentiation
 */
template<typename T>
class Expression {

public:

  /// Define type so we can apply it as a meta-function
  typedef Expression<T> type;

protected:

  // Paul's trick shared pointer, polymorphic root of entire expression tree
  std::shared_ptr<internal::ExpressionNode<T> > root_;

  /// Construct with a custom root
  Expression(const std::shared_ptr<internal::ExpressionNode<T> >& root) : root_(root) {}

public:

  // Expressions wrap trees of functions that can evaluate their own derivatives.
  // The meta-functions below are useful to specify the type of those functions.
  // Example, a function taking a camera and a 3D point and yielding a 2D point:
  //   Expression<Point2>::BinaryFunction<PinholeCamera<Cal3_S2>,Point3>::type
  template<class A1>
  struct UnaryFunction {
    typedef std::function<
        T(const A1&, typename MakeOptionalJacobian<T, A1>::type)> type;
  };

  template<class A1, class A2>
  struct BinaryFunction {
    typedef std::function<
        T(const A1&, const A2&, typename MakeOptionalJacobian<T, A1>::type,
            typename MakeOptionalJacobian<T, A2>::type)> type;
  };

  template<class A1, class A2, class A3>
  struct TernaryFunction {
    typedef std::function<
        T(const A1&, const A2&, const A3&,
            typename MakeOptionalJacobian<T, A1>::type,
            typename MakeOptionalJacobian<T, A2>::type,
            typename MakeOptionalJacobian<T, A3>::type)> type;
  };

  /// Construct a constant expression
  Expression(const T& value);

  /// Construct a leaf expression, with Key
  Expression(const Key& key);

  /// Construct a leaf expression, with Symbol
  Expression(const Symbol& symbol);

  /// Construct a leaf expression, creating Symbol
  Expression(unsigned char c, std::uint64_t j);

  /// Construct a unary function expression
  template<typename A>
  Expression(typename UnaryFunction<A>::type function,
      const Expression<A>& expression);

  /// Construct a binary function expression
  template<typename A1, typename A2>
  Expression(typename BinaryFunction<A1, A2>::type function,
      const Expression<A1>& expression1, const Expression<A2>& expression2);

  /// Construct a ternary function expression
  template<typename A1, typename A2, typename A3>
  Expression(typename TernaryFunction<A1, A2, A3>::type function,
      const Expression<A1>& expression1, const Expression<A2>& expression2,
      const Expression<A3>& expression3);

  /// Construct a nullary method expression
  template<typename A>
  Expression(const Expression<A>& expression,
      T (A::*method)(typename MakeOptionalJacobian<T, A>::type) const);

  /// Construct a unary method expression
  template<typename A1, typename A2>
  Expression(const Expression<A1>& expression1,
      T (A1::*method)(const A2&, typename MakeOptionalJacobian<T, A1>::type,
          typename MakeOptionalJacobian<T, A2>::type) const,
      const Expression<A2>& expression2);

  /// Construct a binary method expression
  template<typename A1, typename A2, typename A3>
  Expression(const Expression<A1>& expression1,
      T (A1::*method)(const A2&, const A3&,
          typename MakeOptionalJacobian<T, A1>::type,
          typename MakeOptionalJacobian<T, A2>::type,
          typename MakeOptionalJacobian<T, A3>::type) const,
      const Expression<A2>& expression2, const Expression<A3>& expression3);

  /// Destructor
  virtual ~Expression() {
  }

  /// Return keys that play in this expression
  std::set<Key> keys() const;

  /// Return dimensions for each argument, as a map
  void dims(std::map<Key, int>& map) const;

  /// Print
  void print(const std::string& s) const;

  /**
   * @brief Return value and optional derivatives, reverse AD version
   * Notes: this is not terribly efficient, and H should have correct size.
   * The order of the Jacobians is same as keys in either keys() or dims()
   */
  T value(const Values& values, std::vector<Matrix>* H = nullptr) const;

  /**
   * An overload of the value function to accept reference to vector of matrices instead of
   * a pointer to vector of matrices.
   */
  T value(const Values& values, std::vector<Matrix>& H) const {
    return value(values, &H);
  }

  /**
   *  @return a "deep" copy of this Expression
   *  "deep" is in quotes because the ExpressionNode hierarchy is *not* cloned.
   *  The intent is for derived classes to be copied using only a Base pointer.
   */
  virtual std::shared_ptr<Expression> clone() const {
    return std::make_shared<Expression>(*this);
  }

  /// Return root
  const std::shared_ptr<internal::ExpressionNode<T> >& root() const;

  /// Return size needed for memory buffer in traceExecution
  size_t traceSize() const;

  /// Add another expression to this expression
  Expression<T>& operator+=(const Expression<T>& e);

protected:

  /// Default constructor, for serialization
  Expression() {}

  /// Keys and dimensions in same order
  typedef std::pair<KeyVector, FastVector<int> > KeysAndDims;
  KeysAndDims keysAndDims() const;

  /// private version that takes keys and dimensions, returns derivatives
  T valueAndDerivatives(const Values& values, const KeyVector& keys,
      const FastVector<int>& dims, std::vector<Matrix>& H) const;

  /// trace execution, very unsafe
  T traceExecution(const Values& values, internal::ExecutionTrace<T>& trace,
      char* traceStorage) const;

  /// brief Return value and derivatives, reverse AD version
  T valueAndJacobianMap(const Values& values,
      internal::JacobianMap& jacobians) const;

  // be very selective on who can access these private methods:
  friend class ExpressionFactor<T> ;
  friend class internal::ExpressionNode<T>;
  friend class ExpressionEqualityConstraint<T>;
  friend class ScalarExpressionInequalityConstraint;

  // and add tests
  friend class ::ExpressionFactorShallowTest;
};

/**
 *  A ScalarMultiplyExpression is a specialization of Expression that multiplies with a scalar
 *  It optimizes the Jacobian calculation for this specific case
 */
template <typename T>
class ScalarMultiplyExpression : public Expression<T> {
  // Check that T is a vector space
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<T>);

 public:
  explicit ScalarMultiplyExpression(double s, const Expression<T>& e);
};

/**
 *  A BinarySumExpression is a specialization of Expression that adds two expressions together
 *  It optimizes the Jacobian calculation for this specific case
 */
template <typename T>
class BinarySumExpression : public Expression<T> {
  // Check that T is a vector space
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<T>);

 public:
  explicit BinarySumExpression(const Expression<T>& e1, const Expression<T>& e2);
};


/**
 * Create an expression out of a linear function f:T->A with (constant) Jacobian dTdA
 *  TODO(frank): create a more efficient version like ScalarMultiplyExpression. This version still
 *  does a malloc every linearize.
 */
template <typename T, typename A>
Expression<T> linearExpression(
    const std::function<T(A)>& f, const Expression<A>& expression,
    const Eigen::Matrix<double, traits<T>::dimension, traits<A>::dimension>& dTdA) {
  // Use lambda to endow f with a linear Jacobian
  typename Expression<T>::template UnaryFunction<A>::type g =
      [=](const A& value, typename MakeOptionalJacobian<T, A>::type H) {
        if (H)
          *H << dTdA;
        return f(value);
      };
  return Expression<T>(g, expression);
}

/**
 *  Construct an expression that executes the scalar multiplication with an input expression
 *  The type T must be a vector space
 *  Example:
 *    Expression<Point2> a(0), b = 12 * a;
 */
template <typename T>
ScalarMultiplyExpression<T> operator*(double s, const Expression<T>& e) {
  return ScalarMultiplyExpression<T>(s, e);
}

/**
 *  Construct an expression that sums two input expressions of the same type T
 *  The type T must be a vector space
 *  Example:
 *    Expression<Point2> a(0), b(1), c = a + b;
 */
template <typename T>
BinarySumExpression<T> operator+(const Expression<T>& e1, const Expression<T>& e2) {
  return BinarySumExpression<T>(e1, e2);
}

/// Construct an expression that subtracts one expression from another
template <typename T>
BinarySumExpression<T> operator-(const Expression<T>& e1, const Expression<T>& e2) {
  // TODO(frank, abe): Implement an actual negate operator instead of multiplying by -1
  return e1 + (-1.0) * e2;
}

/**
 *  Construct a product expression, assumes T::compose(T) -> T
 *  Example:
 *    Expression<Point2> a(0), b(1), c = a*b;
 */
template<typename T>
Expression<T> operator*(const Expression<T>& e1, const Expression<T>& e2);

/**
 *  Construct an array of unknown expressions with successive symbol keys
 *  Example:
 *    createUnknowns<Pose2>(3,'x') creates unknown expressions for x0,x1,x2
 */
template<typename T>
std::vector<Expression<T> > createUnknowns(size_t n, char c, size_t start = 0);

} // namespace gtsam

#include <gtsam/nonlinear/Expression-inl.h>

