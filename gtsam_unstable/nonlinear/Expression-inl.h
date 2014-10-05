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

namespace gtsam {

template<typename T>
class Expression;

//#define NEW_CLASS
#ifdef NEW_CLASS
class JacobianMap: public std::map<Key, Matrix> {
public:
  void add(Key key, const Matrix& H) {
    iterator it = find(key);
    if (it != end())
    it->second += H;
    else
    insert(std::make_pair(key, H));
  }
};
#else
typedef std::map<Key, Matrix> JacobianMap;
#endif

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

  /// Insert terms into jacobians_, premultiplying by H, adding if already exists
  void add(const JacobianMap& terms) {
    BOOST_FOREACH(const Pair& term, terms)
#ifdef NEW_CLASS
    jacobians_.add(term.first, term.second);
#else
    {
      JacobianMap::iterator it = jacobians_.find(term.first);
      if (it != jacobians_.end())
        it->second += term.second;
      else
        jacobians_[term.first] = term.second;
    }
#endif
  }

  /// Insert terms into jacobians_, premultiplying by H, adding if already exists
  void add(const Matrix& H, const JacobianMap& terms) {
    BOOST_FOREACH(const Pair& term, terms)
#ifdef NEW_CLASS
    jacobians_.add(term.first, H * term.second);
#else
    {
      JacobianMap::iterator it = jacobians_.find(term.first);
      if (it != jacobians_.end())
        it->second += H * term.second;
      else
        jacobians_[term.first] = H * term.second;
    }
#endif
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

  /// Construct value dependent on a single key, with Jacobain H
  Augmented(const T& t, Key key, const Matrix& H) :
      value_(t) {
    jacobians_[key] = H;
  }

  /// Construct value, pre-multiply jacobians by H
  Augmented(const T& t, const Matrix& H, const JacobianMap& jacobians) :
      value_(t) {
    add(H, jacobians);
  }

  /// Construct value, pre-multiply jacobians by H
  Augmented(const T& t, const Matrix& H1, const JacobianMap& jacobians1,
      const Matrix& H2, const JacobianMap& jacobians2) :
      value_(t) {
    add(H1, jacobians1);
    add(H2, jacobians2);
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
template<class T>
struct JacobianTrace {
  T t;
  T value() const {
    return t;
  }
  virtual void reverseAD(JacobianMap& jacobians) const = 0;
  virtual void reverseAD(const Matrix& H, JacobianMap& jacobians) const = 0;
};

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
  virtual boost::shared_ptr<JacobianTrace<T> > traceExecution(
      const Values& values) const = 0;
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
  struct Trace: public JacobianTrace<T> {
    /// If the expression is just a constant, we do nothing
    virtual void reverseAD(JacobianMap& jacobians) const {
    }
    /// Base case: we simply ignore the given df/dT
    virtual void reverseAD(const Matrix& H, JacobianMap& jacobians) const {
    }
  };

  /// Construct an execution trace for reverse AD
  virtual boost::shared_ptr<JacobianTrace<T> > traceExecution(
      const Values& values) const {
    boost::shared_ptr<Trace> trace = boost::make_shared<Trace>();
    trace->t = constant_;
    return trace;
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
    T t = value(values);
    return Augmented<T>(t, key_);
  }

  /// Trace structure for reverse AD
  struct Trace: public JacobianTrace<T> {
    Key key;
    /// If the expression is just a leaf, we just insert an identity matrix
    virtual void reverseAD(JacobianMap& jacobians) const {
      size_t n = T::Dim();
      jacobians[key] = Eigen::MatrixXd::Identity(n, n);
    }
    /// Base case: given df/dT, add it jacobians with correct key and we are done
    virtual void reverseAD(const Matrix& H, JacobianMap& jacobians) const {
#ifdef NEW_CLASS
      jacobians.add(key, H);
#else
      JacobianMap::iterator it = jacobians.find(key);
      if (it != jacobians.end())
        it->second += H;
      else
        jacobians[key] = H;
#endif
    }
  };

  /// Construct an execution trace for reverse AD
  virtual boost::shared_ptr<JacobianTrace<T> > traceExecution(
      const Values& values) const {
    boost::shared_ptr<Trace> trace = boost::make_shared<Trace>();
    trace->t = value(values);
    trace->key = key_;
    return trace;
  }

};

//-----------------------------------------------------------------------------
/// Unary Function Expression
template<class T, class A>
class UnaryExpression: public ExpressionNode<T> {

public:

  typedef boost::function<T(const A&, boost::optional<Matrix&>)> Function;

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
    Matrix H;
    T t = function_(argument.value(),
        argument.constant() ? none : boost::optional<Matrix&>(H));
    return Augmented<T>(t, H, argument.jacobians());
  }

  /// Trace structure for reverse AD
  struct Trace: public JacobianTrace<T> {
    boost::shared_ptr<JacobianTrace<A> > trace1;
    Matrix H1;
    /// Start the reverse AD process
    virtual void reverseAD(JacobianMap& jacobians) const {
      trace1->reverseAD(H1, jacobians);
    }
    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    virtual void reverseAD(const Matrix& H, JacobianMap& jacobians) const {
      trace1->reverseAD(H * H1, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  virtual boost::shared_ptr<JacobianTrace<T> > traceExecution(
      const Values& values) const {
    boost::shared_ptr<Trace> trace = boost::make_shared<Trace>();
    trace->trace1 = this->expressionA_->traceExecution(values);
    trace->t = function_(trace->trace1->value(), trace->H1);
    return trace;
  }
};

//-----------------------------------------------------------------------------
/// Binary Expression

template<class T, class A1, class A2>
class BinaryExpression: public ExpressionNode<T> {

public:

  typedef boost::function<
      T(const A1&, const A2&, boost::optional<Matrix&>,
          boost::optional<Matrix&>)> Function;

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
    Augmented<A1> argument1 = this->expressionA1_->forward(values);
    Augmented<A2> argument2 = this->expressionA2_->forward(values);
    Matrix H1, H2;
    T t = function_(argument1.value(), argument2.value(),
        argument1.constant() ? none : boost::optional<Matrix&>(H1),
        argument2.constant() ? none : boost::optional<Matrix&>(H2));
    return Augmented<T>(t, H1, argument1.jacobians(), H2, argument2.jacobians());
  }

  /// Trace structure for reverse AD
  struct Trace: public JacobianTrace<T> {
    boost::shared_ptr<JacobianTrace<A1> > trace1;
    boost::shared_ptr<JacobianTrace<A2> > trace2;
    Matrix H1, H2;
    /// Start the reverse AD process
    virtual void reverseAD(JacobianMap& jacobians) const {
      trace1->reverseAD(H1, jacobians);
      trace2->reverseAD(H2, jacobians);
    }
    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    virtual void reverseAD(const Matrix& H, JacobianMap& jacobians) const {
      trace1->reverseAD(H * H1, jacobians);
      trace2->reverseAD(H * H2, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  virtual boost::shared_ptr<JacobianTrace<T> > traceExecution(
      const Values& values) const {
    boost::shared_ptr<Trace> trace = boost::make_shared<Trace>();
    trace->trace1 = this->expressionA1_->traceExecution(values);
    trace->trace2 = this->expressionA2_->traceExecution(values);
    trace->t = function_(trace->trace1->value(), trace->trace2->value(),
        trace->H1, trace->H2);
    return trace;
  }

};

//-----------------------------------------------------------------------------

}

