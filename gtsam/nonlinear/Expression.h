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

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <map>

// Forward declare tests
class ExpressionFactorShallowTest;

namespace gtsam {

// Forward declares
class Values;
template<typename T> class ExpressionFactor;

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

private:

  // Paul's trick shared pointer, polymorphic root of entire expression tree
  boost::shared_ptr<internal::ExpressionNode<T> > root_;

public:

  // Expressions wrap trees of functions that can evaluate their own derivatives.
  // The meta-functions below are useful to specify the type of those functions.
  // Example, a function taking a camera and a 3D point and yielding a 2D point:
  //   Expression<Point2>::BinaryFunction<SimpleCamera,Point3>::type
  template<class A1>
  struct UnaryFunction {
    typedef boost::function<
        T(const A1&, typename MakeOptionalJacobian<T, A1>::type)> type;
  };

  template<class A1, class A2>
  struct BinaryFunction {
    typedef boost::function<
        T(const A1&, const A2&, typename MakeOptionalJacobian<T, A1>::type,
            typename MakeOptionalJacobian<T, A2>::type)> type;
  };

  template<class A1, class A2, class A3>
  struct TernaryFunction {
    typedef boost::function<
        T(const A1&, const A2&, const A3&,
            typename MakeOptionalJacobian<T, A1>::type,
            typename MakeOptionalJacobian<T, A2>::type,
            typename MakeOptionalJacobian<T, A3>::type)> type;
  };

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

  /// Return root
  const boost::shared_ptr<internal::ExpressionNode<T> >& root() const;

  /// Return size needed for memory buffer in traceExecution
  size_t traceSize() const;

private:

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
      void* traceStorage) const;

  /// brief Return value and derivatives, reverse AD version
  T valueAndJacobianMap(const Values& values,
      internal::JacobianMap& jacobians) const;

  // be very selective on who can access these private methods:
  friend class ExpressionFactor<T> ;
  friend class internal::ExpressionNode<T>;

  // and add tests
  friend class ::ExpressionFactorShallowTest;
};

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

