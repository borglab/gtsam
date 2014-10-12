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
#include <gtsam/base/Testable.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <new> // for placement new
struct TestBinaryExpression;

// template meta-programming headers
#include <boost/mpl/vector.hpp>
#include <boost/mpl/plus.hpp>
#include <boost/mpl/front.hpp>
#include <boost/mpl/pop_front.hpp>
#include <boost/mpl/fold.hpp>
#include <boost/mpl/empty_base.hpp>
#include <boost/mpl/placeholders.hpp>

namespace MPL = boost::mpl::placeholders;

namespace gtsam {

template<typename T>
class Expression;

typedef std::map<Key, Matrix> JacobianMap;

//-----------------------------------------------------------------------------
/**
 * The CallRecord class stores the Jacobians of applying a function
 * with respect to each of its arguments. It also stores an executation trace
 * (defined below) for each of its arguments.
 *
 * It is sub-classed in the function-style ExpressionNode sub-classes below.
 */
template<int COLS>
struct CallRecord {
  virtual void print(const std::string& indent) const {
  }
  virtual void startReverseAD(JacobianMap& jacobians) const {
  }
  virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const {
  }
  typedef Eigen::Matrix<double, 2, COLS> Jacobian2T;
  virtual void reverseAD2(const Jacobian2T& dFdT,
      JacobianMap& jacobians) const {
  }
};

//-----------------------------------------------------------------------------
/**
 * The ExecutionTrace class records a tree-structured expression's execution
 * It is a tagged union that obviates the need to create
 * a ExecutionTrace subclass for Constants and Leaf Expressions. Instead
 * the key for the leaf is stored in the space normally used to store a
 * CallRecord*. Nothing is stored for a Constant.
 */
template<class T>
class ExecutionTrace {
  enum {
    Constant, Leaf, Function
  } type;
  union {
    Key key;
    CallRecord<T::dimension>* ptr;
  } content;
public:
  /// Pointer always starts out as a Constant
  ExecutionTrace() :
      type(Constant) {
  }
  /// Change pointer to a Leaf Record
  void setLeaf(Key key) {
    type = Leaf;
    content.key = key;
  }
  /// Take ownership of pointer to a Function Record
  void setFunction(CallRecord<T::dimension>* record) {
    type = Function;
    content.ptr = record;
  }
  /// Print
  void print(const std::string& indent = "") const {
    if (type == Constant)
      std::cout << indent << "Constant" << std::endl;
    else if (type == Leaf)
      std::cout << indent << "Leaf, key = " << content.key << std::endl;
    else if (type == Function) {
      std::cout << indent << "Function" << std::endl;
      content.ptr->print(indent + "  ");
    }
  }
  /// Return record pointer, quite unsafe, used only for testing
  template<class Record>
  boost::optional<Record*> record() {
    if (type != Function)
      return boost::none;
    else {
      Record* p = dynamic_cast<Record*>(content.ptr);
      return p ? boost::optional<Record*>(p) : boost::none;
    }
  }
  // *** This is the main entry point for reverseAD, called from Expression::augmented ***
  // Called only once, either inserts identity into Jacobians (Leaf) or starts AD (Function)
  void startReverseAD(JacobianMap& jacobians) const {
    if (type == Leaf) {
      // This branch will only be called on trivial Leaf expressions, i.e. Priors
      size_t n = T::Dim();
      jacobians[content.key] = Eigen::MatrixXd::Identity(n, n);
    } else if (type == Function)
      // This is the more typical entry point, starting the AD pipeline
      // It is inside the startReverseAD that the correctly dimensioned pipeline is chosen.
      content.ptr->startReverseAD(jacobians);
  }
  // Either add to Jacobians (Leaf) or propagate (Function)
  void reverseAD(const Matrix& dTdA, JacobianMap& jacobians) const {
    if (type == Leaf) {
      JacobianMap::iterator it = jacobians.find(content.key);
      if (it != jacobians.end())
        it->second += dTdA;
      else
        jacobians[content.key] = dTdA;
    } else if (type == Function)
      content.ptr->reverseAD(dTdA, jacobians);
  }
  // Either add to Jacobians (Leaf) or propagate (Function)
  typedef Eigen::Matrix<double, 2, T::dimension> Jacobian2T;
  void reverseAD2(const Jacobian2T& dTdA, JacobianMap& jacobians) const {
    if (type == Leaf) {
      JacobianMap::iterator it = jacobians.find(content.key);
      if (it != jacobians.end())
        it->second += dTdA;
      else
        jacobians[content.key] = dTdA;
    } else if (type == Function)
      content.ptr->reverseAD2(dTdA, jacobians);
  }
};

/// Primary template calls the generic Matrix reverseAD pipeline
template<size_t M, class A>
struct Select {
  typedef Eigen::Matrix<double, M, A::dimension> Jacobian;
  static void reverseAD(const ExecutionTrace<A>& trace, const Jacobian& dTdA,
      JacobianMap& jacobians) {
    trace.reverseAD(dTdA, jacobians);
  }
};

/// Partially specialized template calls the 2-dimensional output version
template<class A>
struct Select<2, A> {
  typedef Eigen::Matrix<double, 2, A::dimension> Jacobian;
  static void reverseAD(const ExecutionTrace<A>& trace, const Jacobian& dTdA,
      JacobianMap& jacobians) {
    trace.reverseAD2(dTdA, jacobians);
  }
};

//-----------------------------------------------------------------------------
/**
 * Record the evaluation of a single argument in a functional expression
 * Building block for Recursive Record Class
 */
template<class T, class A, size_t N>
struct Argument {
  typedef Eigen::Matrix<double, T::dimension, A::dimension> JacobianTA;
  ExecutionTrace<A> trace;
  JacobianTA dTdA;
};

/**
 * Recursive Record Class for Functional Expressions
 * Abrahams, David; Gurtovoy, Aleksey (2004-12-10).
 * C++ Template Metaprogramming: Concepts, Tools, and Techniques from Boost
 * and Beyond. Pearson Education.
 */
template<class T, class AN, class More>
struct Record: Argument<T, typename AN::type, AN::value>, More {

  typedef T return_type;
  typedef typename AN::type A;
  const static size_t N = AN::value;
  typedef Argument<T, A, N> This;

  /// Print to std::cout
  virtual void print(const std::string& indent) const {
    More::print(indent);
    static const Eigen::IOFormat matlab(0, 1, " ", "; ", "", "", "[", "]");
    std::cout << This::dTdA.format(matlab) << std::endl;
    This::trace.print(indent);
  }

  /// Start the reverse AD process
  virtual void startReverseAD(JacobianMap& jacobians) const {
    More::startReverseAD(jacobians);
    Select<T::dimension, A>::reverseAD(This::trace, This::dTdA, jacobians);
  }

  /// Given df/dT, multiply in dT/dA and continue reverse AD process
  virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const {
    More::reverseAD(dFdT, jacobians);
    This::trace.reverseAD(dFdT * This::dTdA, jacobians);
  }

  /// Version specialized to 2-dimensional output
  typedef Eigen::Matrix<double, 2, T::dimension> Jacobian2T;
  virtual void reverseAD2(const Jacobian2T& dFdT,
      JacobianMap& jacobians) const {
    More::reverseAD2(dFdT, jacobians);
    This::trace.reverseAD2(dFdT * This::dTdA, jacobians);
  }
};

/// Meta-function for generating a numbered type
template<class A, size_t N>
struct Numbered {
  typedef A type;
  typedef size_t value_type;
  static const size_t value = N;
};

/// Recursive Record class Generator
template<class T, class TYPES>
struct GenerateRecord {
  typedef typename boost::mpl::fold<TYPES, CallRecord<T::dimension>,
      Record<T, MPL::_2, MPL::_1> >::type type;
};

/// Access Argument
template<class A, size_t N, class Record>
Argument<typename Record::return_type, A, N>& argument(Record& record) {
  return static_cast<Argument<typename Record::return_type, A, N>&>(record);
}

/// Access Trace
template<class A, size_t N, class Record>
ExecutionTrace<A>& getTrace(Record* record) {
  return argument<A, N>(*record).trace;
}

/// Access Jacobian
template<class A, size_t N, class Record>
Eigen::Matrix<double, Record::return_type::dimension, A::dimension>& jacobian(
    Record* record) {
  return argument<A, N>(*record).dTdA;
}

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

  /// Move terms to array, destroys content
  void move(std::vector<Matrix>& H) {
    assert(H.size()==jacobians_.size());
    size_t j = 0;
    JacobianMap::iterator it = jacobians_.begin();
    for (; it != jacobians_.end(); ++it)
      it->second.swap(H[j++]);
  }

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

  // Return size needed for memory buffer in traceExecution
  virtual size_t traceSize() const {
    return 0;
  }

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const = 0;
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

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const {
    return constant_;
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

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const {
    trace.setLeaf(key_);
    return values.at<T>(key_);
  }

};

//-----------------------------------------------------------------------------
/// Unary Function Expression
template<class T, class A1>
class UnaryExpression: public ExpressionNode<T> {

public:

  typedef Eigen::Matrix<double, T::dimension, A1::dimension> JacobianTA;
  typedef boost::function<T(const A1&, boost::optional<JacobianTA&>)> Function;

private:

  Function function_;
  boost::shared_ptr<ExpressionNode<A1> > expressionA1_;

  /// Constructor with a unary function f, and input argument e
  UnaryExpression(Function f, const Expression<A1>& e) :
      function_(f), expressionA1_(e.root()) {
  }

  friend class Expression<T> ;

public:

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    return expressionA1_->keys();
  }

  /// Return value
  virtual T value(const Values& values) const {
    return function_(this->expressionA1_->value(values), boost::none);
  }

  /// Return value and derivatives
  virtual Augmented<T> forward(const Values& values) const {
    using boost::none;
    Augmented<A1> argument = this->expressionA1_->forward(values);
    JacobianTA dTdA;
    T t = function_(argument.value(),
        argument.constant() ? none : boost::optional<JacobianTA&>(dTdA));
    return Augmented<T>(t, dTdA, argument.jacobians());
  }

  /// CallRecord structure for reverse AD
  typedef boost::mpl::vector<Numbered<A1, 1> > Arguments;
  typedef typename GenerateRecord<T, Arguments>::type Record;

  // Return size needed for memory buffer in traceExecution
  virtual size_t traceSize() const {
    return sizeof(Record) + expressionA1_->traceSize();
  }

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const {
    Record* record = new (raw) Record();
    trace.setFunction(record);

    raw = (char*) (record + 1);
    A1 a1 = expressionA1_->traceExecution(values, getTrace<A1, 1>(record), raw);

    return function_(a1, jacobian<A1, 1>(record));
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
  friend struct ::TestBinaryExpression;

public:

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

  /// CallRecord structure for reverse AD
  typedef boost::mpl::vector<Numbered<A1, 1>, Numbered<A2, 2> > Arguments;
  typedef typename GenerateRecord<T, Arguments>::type Record;

  // Return size needed for memory buffer in traceExecution
  virtual size_t traceSize() const {
    return sizeof(Record) + expressionA1_->traceSize()
        + expressionA2_->traceSize();
  }

  /// Construct an execution trace for reverse AD
  /// The raw buffer is [Record | A1 raw | A2 raw]
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const {
    Record* record = new (raw) Record();
    trace.setFunction(record);

    raw = (char*) (record + 1);
    A1 a1 = expressionA1_->traceExecution(values, getTrace<A1, 1>(record), raw);
    raw = raw + expressionA1_->traceSize();
    A2 a2 = expressionA2_->traceExecution(values, getTrace<A2, 2>(record), raw);

    return function_(a1, a2, jacobian<A1, 1>(record), jacobian<A2, 2>(record));
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

  /// CallRecord structure for reverse AD
  typedef boost::mpl::vector<Numbered<A1, 1>, Numbered<A2, 2>, Numbered<A3, 3> > Arguments;
  typedef typename GenerateRecord<T, Arguments>::type Record;

  // Return size needed for memory buffer in traceExecution
  virtual size_t traceSize() const {
    return sizeof(Record) + expressionA1_->traceSize()
        + expressionA2_->traceSize() + expressionA2_->traceSize();
  }

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const {
    Record* record = new (raw) Record();
    trace.setFunction(record);

    raw = (char*) (record + 1);
    A1 a1 = expressionA1_->traceExecution(values, getTrace<A1, 1>(record), raw);
    raw = raw + expressionA1_->traceSize();
    A2 a2 = expressionA2_->traceExecution(values, getTrace<A2, 2>(record), raw);
    raw = raw + expressionA2_->traceSize();
    A3 a3 = expressionA3_->traceExecution(values, getTrace<A3, 3>(record), raw);

    return function_(a1, a2, a3, jacobian<A1, 1>(record),
        jacobian<A2, 2>(record), jacobian<A3, 3>(record));
  }

};
//-----------------------------------------------------------------------------
}

