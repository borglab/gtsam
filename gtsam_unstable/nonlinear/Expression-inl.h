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
#include <boost/tuple/tuple.hpp>

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

  /// Insert terms into jacobians_, adding if already exists
  void add(const JacobianMap& terms) {
    BOOST_FOREACH(const Pair& term, terms) {
      JacobianMap::iterator it = jacobians_.find(term.first);
      if (it != jacobians_.end())
        it->second += term.second;
      else
        jacobians_[term.first] = term.second;
    }
  }

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

  /// Construct value, pre-multiply jacobians by dTdA
  Augmented(const T& t, const Matrix& dTdA, const JacobianMap& jacobians) :
      value_(t) {
    add(dTdA, jacobians);
  }

  /// Construct value, pre-multiply jacobians
  Augmented(const T& t, const Matrix& dTdA1, const JacobianMap& jacobians1,
      const Matrix& dTdA2, const JacobianMap& jacobians2) :
      value_(t) {
    add(dTdA1, jacobians1);
    add(dTdA2, jacobians2);
  }

  /// Construct value, pre-multiply jacobians
  Augmented(const T& t, const Matrix& dTdA1, const JacobianMap& jacobians1,
      const Matrix& dTdA2, const JacobianMap& jacobians2, const Matrix& dTdA3,
      const JacobianMap& jacobians3) :
      value_(t) {
    add(dTdA1, jacobians1);
    add(dTdA2, jacobians2);
    add(dTdA3, jacobians3);
  }

  /// Return value
  const T& value() const {
    return value_;
  }

  /// Return jacobians
  const JacobianMap& jacobians() const {
    return jacobians_;
  }

  /// Return jacobians
  JacobianMap& jacobians() {
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
struct JacobianTrace {
  virtual void reverseAD(JacobianMap& jacobians) const = 0;
  virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const = 0;
//  template<class JacobianFT>
//  void reverseAD(const JacobianFT& dFdT, JacobianMap& jacobians) const {
};

typedef boost::shared_ptr<JacobianTrace> TracePtr;

//template <class Derived>
//struct TypedTrace {
//  virtual void reverseAD(JacobianMap& jacobians) const = 0;
//  virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const = 0;
////  template<class JacobianFT>
////  void reverseAD(const JacobianFT& dFdT, JacobianMap& jacobians) const {
//};

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
  virtual Augmented<T> forward(const Values& values) const = 0;

  /// Construct an execution trace for reverse AD
  virtual std::pair<T, TracePtr> traceExecution(const Values& values) const = 0;
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
  virtual Augmented<T> forward(const Values& values) const {
    return Augmented<T>(constant_);
  }

  /// Trace structure for reverse AD
  struct Trace: public JacobianTrace {
    /// If the expression is just a constant, we do nothing
    virtual void reverseAD(JacobianMap& jacobians) const {
    }
    /// Base case: we simply ignore the given df/dT
    virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const {
    }
  };

  /// Construct an execution trace for reverse AD
  virtual std::pair<T, TracePtr> traceExecution(const Values& values) const {
    boost::shared_ptr<Trace> trace = boost::make_shared<Trace>();
    return std::make_pair(constant_, trace);
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
  virtual Augmented<T> forward(const Values& values) const {
    return Augmented<T>(values.at<T>(key_), key_);
  }

  /// Trace structure for reverse AD
  struct Trace: public JacobianTrace {
    Key key;
    /// If the expression is just a leaf, we just insert an identity matrix
    virtual void reverseAD(JacobianMap& jacobians) const {
      size_t n = T::Dim();
      jacobians[key] = Eigen::MatrixXd::Identity(n, n);
    }
    /// Base case: given df/dT, add it jacobians with correct key and we are done
    virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const {
      JacobianMap::iterator it = jacobians.find(key);
      if (it != jacobians.end())
        it->second += dFdT;
      else
        jacobians[key] = dFdT;
    }
  };

  /// Construct an execution trace for reverse AD
  virtual std::pair<T, TracePtr> traceExecution(const Values& values) const {
    boost::shared_ptr<Trace> trace = boost::make_shared<Trace>();
    trace->key = key_;
    return std::make_pair(values.at<T>(key_), trace);
  }

};

//-----------------------------------------------------------------------------
/// Unary Function Expression
template<class T, class A>
class UnaryExpression: public ExpressionNode<T> {

public:

  typedef Eigen::Matrix<double, T::dimension, A::dimension> JacobianTA;
  typedef boost::function<T(const A&, boost::optional<JacobianTA&>)> Function;

private:

  Function function_;
  boost::shared_ptr<ExpressionNode<A> > expressionA_;

  /// Constructor with a unary function f, and input argument e
  UnaryExpression(Function f, const Expression<A>& e) :
      function_(f), expressionA_(e.root()) {
  }

  friend class Expression<T> ;

public:

  /// Destructor
  virtual ~UnaryExpression() {
  }

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    return expressionA_->keys();
  }

  /// Return value
  virtual T value(const Values& values) const {
    return function_(this->expressionA_->value(values), boost::none);
  }

  /// Return value and derivatives
  virtual Augmented<T> forward(const Values& values) const {
    using boost::none;
    Augmented<A> argument = this->expressionA_->forward(values);
    JacobianTA dTdA;
    T t = function_(argument.value(),
        argument.constant() ? none : boost::optional<JacobianTA&>(dTdA));
    return Augmented<T>(t, dTdA, argument.jacobians());
  }

  /// Trace structure for reverse AD
  struct Trace: public JacobianTrace {
    TracePtr trace;
    JacobianTA dTdA;
    /// Start the reverse AD process
    virtual void reverseAD(JacobianMap& jacobians) const {
      trace->reverseAD(dTdA, jacobians);
    }
    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const {
      trace->reverseAD(dFdT * dTdA, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  virtual std::pair<T, TracePtr> traceExecution(const Values& values) const {
    A a;
    boost::shared_ptr<Trace> trace = boost::make_shared<Trace>();
    boost::tie(a, trace->trace) = this->expressionA_->traceExecution(values);
    return std::make_pair(function_(a, trace->dTdA),trace);
  }
};

//-----------------------------------------------------------------------------
/// Binary Expression

template<class T, class A1, class A2>
class BinaryExpression: public ExpressionNode<T> {

public:

  typedef Eigen::Matrix<double, T::dimension, A1::dimension> JacobianTA1;
  typedef Eigen::Matrix<double, T::dimension, A2::dimension> JacobianTA2;
  typedef boost::function<
      T(const A1&, const A2&, boost::optional<JacobianTA1&>,
          boost::optional<JacobianTA2&>)> Function;

private:

  Function function_;
  boost::shared_ptr<ExpressionNode<A1> > expressionA1_;
  boost::shared_ptr<ExpressionNode<A2> > expressionA2_;

  /// Constructor with a binary function f, and two input arguments
  BinaryExpression(Function f, //
      const Expression<A1>& e1, const Expression<A2>& e2) :
      function_(f), expressionA1_(e1.root()), expressionA2_(e2.root()) {
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
    return function_(this->expressionA1_->value(values),
        this->expressionA2_->value(values), none, none);
  }

  /// Return value and derivatives
  virtual Augmented<T> forward(const Values& values) const {
    using boost::none;
    Augmented<A1> a1 = this->expressionA1_->forward(values);
    Augmented<A2> a2 = this->expressionA2_->forward(values);
    JacobianTA1 dTdA1;
    JacobianTA2 dTdA2;
    T t = function_(a1.value(), a2.value(),
        a1.constant() ? none : boost::optional<JacobianTA1&>(dTdA1),
        a2.constant() ? none : boost::optional<JacobianTA2&>(dTdA2));
    return Augmented<T>(t, dTdA1, a1.jacobians(), dTdA2, a2.jacobians());
  }

  /// Trace structure for reverse AD
  struct Trace: public JacobianTrace {
    TracePtr trace1, trace2;
    JacobianTA1 dTdA1;
    JacobianTA2 dTdA2;
    /// Start the reverse AD process
    virtual void reverseAD(JacobianMap& jacobians) const {
      trace1->reverseAD(dTdA1, jacobians);
      trace2->reverseAD(dTdA2, jacobians);
    }
    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const {
      trace1->reverseAD(dFdT * dTdA1, jacobians);
      trace2->reverseAD(dFdT * dTdA2, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  virtual std::pair<T, TracePtr> traceExecution(const Values& values) const {
    A1 a1;
    A2 a2;
    boost::shared_ptr<Trace> trace = boost::make_shared<Trace>();
    boost::tie(a1, trace->trace1) = this->expressionA1_->traceExecution(values);
    boost::tie(a2, trace->trace2) = this->expressionA2_->traceExecution(values);
    return std::make_pair(function_(a1, a2, trace->dTdA1, trace->dTdA2), trace);
  }

};

//-----------------------------------------------------------------------------
/// Ternary Expression

template<class T, class A1, class A2, class A3>
class TernaryExpression: public ExpressionNode<T> {

public:

  typedef Eigen::Matrix<double, T::dimension, A1::dimension> JacobianTA1;
  typedef Eigen::Matrix<double, T::dimension, A2::dimension> JacobianTA2;
  typedef Eigen::Matrix<double, T::dimension, A3::dimension> JacobianTA3;
  typedef boost::function<
      T(const A1&, const A2&, const A3&, boost::optional<JacobianTA1&>,
          boost::optional<JacobianTA2&>, boost::optional<JacobianTA3&>)> Function;

private:

  Function function_;
  boost::shared_ptr<ExpressionNode<A1> > expressionA1_;
  boost::shared_ptr<ExpressionNode<A2> > expressionA2_;
  boost::shared_ptr<ExpressionNode<A3> > expressionA3_;

  /// Constructor with a ternary function f, and three input arguments
  TernaryExpression(
      Function f, //
      const Expression<A1>& e1, const Expression<A2>& e2,
      const Expression<A3>& e3) :
      function_(f), expressionA1_(e1.root()), expressionA2_(e2.root()), expressionA3_(
          e3.root()) {
  }

  friend class Expression<T> ;

public:

  /// Destructor
  virtual ~TernaryExpression() {
  }

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    std::set<Key> keys1 = expressionA1_->keys();
    std::set<Key> keys2 = expressionA2_->keys();
    std::set<Key> keys3 = expressionA3_->keys();
    keys2.insert(keys3.begin(), keys3.end());
    keys1.insert(keys2.begin(), keys2.end());
    return keys1;
  }

  /// Return value
  virtual T value(const Values& values) const {
    using boost::none;
    return function_(this->expressionA1_->value(values),
        this->expressionA2_->value(values), this->expressionA3_->value(values),
        none, none, none);
  }

  /// Return value and derivatives
  virtual Augmented<T> forward(const Values& values) const {
    using boost::none;
    Augmented<A1> a1 = this->expressionA1_->forward(values);
    Augmented<A2> a2 = this->expressionA2_->forward(values);
    Augmented<A3> a3 = this->expressionA3_->forward(values);
    JacobianTA1 dTdA1;
    JacobianTA2 dTdA2;
    JacobianTA3 dTdA3;
    T t = function_(a1.value(), a2.value(), a3.value(),
        a1.constant() ? none : boost::optional<JacobianTA1&>(dTdA1),
        a2.constant() ? none : boost::optional<JacobianTA2&>(dTdA2),
        a3.constant() ? none : boost::optional<JacobianTA3&>(dTdA3));
    return Augmented<T>(t, dTdA1, a1.jacobians(), dTdA2, a2.jacobians(), dTdA3,
        a3.jacobians());
  }

  /// Trace structure for reverse AD
  struct Trace: public JacobianTrace {
    TracePtr trace1;
    TracePtr trace2;
    TracePtr trace3;
    JacobianTA1 dTdA1;
    JacobianTA2 dTdA2;
    JacobianTA3 dTdA3;
    /// Start the reverse AD process
    virtual void reverseAD(JacobianMap& jacobians) const {
      trace1->reverseAD(dTdA1, jacobians);
      trace2->reverseAD(dTdA2, jacobians);
      trace3->reverseAD(dTdA3, jacobians);
    }
    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const {
      trace1->reverseAD(dFdT * dTdA1, jacobians);
      trace2->reverseAD(dFdT * dTdA2, jacobians);
      trace3->reverseAD(dFdT * dTdA3, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  virtual std::pair<T, TracePtr> traceExecution(const Values& values) const {
    A1 a1;
    A2 a2;
    A3 a3;
    boost::shared_ptr<Trace> trace = boost::make_shared<Trace>();
    boost::tie(a1, trace->trace1) = this->expressionA1_->traceExecution(values);
    boost::tie(a2, trace->trace2) = this->expressionA2_->traceExecution(values);
    boost::tie(a3, trace->trace3) = this->expressionA3_->traceExecution(values);
    return std::make_pair(
        function_(a1, a2, a3, trace->dTdA1, trace->dTdA2, trace->dTdA3), trace);
  }

};
//-----------------------------------------------------------------------------
}

