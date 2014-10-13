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

class ExpressionFactorBinaryTest;
// Forward declare for testing

namespace gtsam {

template<typename T>
class Expression;

typedef std::map<Key, Matrix> JacobianMap;

/// Move terms to array, destroys content
void move(JacobianMap& jacobians, std::vector<Matrix>& H) {
  assert(H.size()==jacobians.size());
  size_t j = 0;
  JacobianMap::iterator it = jacobians.begin();
  for (; it != jacobians.end(); ++it)
    it->second.swap(H[j++]);
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
    move(jacobians_, H);
  }

};

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
  static size_t const N = 0;
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
  } kind;
  union {
    Key key;
    CallRecord<T::dimension>* ptr;
  } content;
public:
  /// Pointer always starts out as a Constant
  ExecutionTrace() :
      kind(Constant) {
  }
  /// Change pointer to a Leaf Record
  void setLeaf(Key key) {
    kind = Leaf;
    content.key = key;
  }
  /// Take ownership of pointer to a Function Record
  void setFunction(CallRecord<T::dimension>* record) {
    kind = Function;
    content.ptr = record;
  }
  /// Print
  void print(const std::string& indent = "") const {
    if (kind == Constant)
      std::cout << indent << "Constant" << std::endl;
    else if (kind == Leaf)
      std::cout << indent << "Leaf, key = " << content.key << std::endl;
    else if (kind == Function) {
      std::cout << indent << "Function" << std::endl;
      content.ptr->print(indent + "  ");
    }
  }
  /// Return record pointer, quite unsafe, used only for testing
  template<class Record>
  boost::optional<Record*> record() {
    if (kind != Function)
      return boost::none;
    else {
      Record* p = dynamic_cast<Record*>(content.ptr);
      return p ? boost::optional<Record*>(p) : boost::none;
    }
  }
  // *** This is the main entry point for reverseAD, called from Expression::augmented ***
  // Called only once, either inserts identity into Jacobians (Leaf) or starts AD (Function)
  void startReverseAD(JacobianMap& jacobians) const {
    if (kind == Leaf) {
      // This branch will only be called on trivial Leaf expressions, i.e. Priors
      size_t n = T::Dim();
      jacobians[content.key] = Eigen::MatrixXd::Identity(n, n);
    } else if (kind == Function)
      // This is the more typical entry point, starting the AD pipeline
      // It is inside the startReverseAD that the correctly dimensioned pipeline is chosen.
      content.ptr->startReverseAD(jacobians);
  }
  // Either add to Jacobians (Leaf) or propagate (Function)
  void reverseAD(const Matrix& dTdA, JacobianMap& jacobians) const {
    if (kind == Leaf) {
      JacobianMap::iterator it = jacobians.find(content.key);
      if (it != jacobians.end())
        it->second += dTdA;
      else
        jacobians[content.key] = dTdA;
    } else if (kind == Function)
      content.ptr->reverseAD(dTdA, jacobians);
  }
  // Either add to Jacobians (Leaf) or propagate (Function)
  typedef Eigen::Matrix<double, 2, T::dimension> Jacobian2T;
  void reverseAD2(const Jacobian2T& dTdA, JacobianMap& jacobians) const {
    if (kind == Leaf) {
      JacobianMap::iterator it = jacobians.find(content.key);
      if (it != jacobians.end())
        it->second += dTdA;
      else
        jacobians[content.key] = dTdA;
    } else if (kind == Function)
      content.ptr->reverseAD2(dTdA, jacobians);
  }

  /// Define type so we can apply it as a meta-function
  typedef ExecutionTrace<T> type;
};

/// Primary template calls the generic Matrix reverseAD pipeline
template<size_t ROWS, class A>
struct Select {
  typedef Eigen::Matrix<double, ROWS, A::dimension> Jacobian;
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
 * Expression node. The superclass for objects that do the heavy lifting
 * An Expression<T> has a pointer to an ExpressionNode<T> underneath
 * allowing Expressions to have polymorphic behaviour even though they
 * are passed by value. This is the same way boost::function works.
 * http://loki-lib.sourceforge.net/html/a00652.html
 */
template<class T>
class ExpressionNode {

protected:

  size_t traceSize_;

  /// Constructor, traceSize is size of the execution trace of expression rooted here
  ExpressionNode(size_t traceSize = 0) :
      traceSize_(traceSize) {
  }

public:

  /// Destructor
  virtual ~ExpressionNode() {
  }

  /// Return keys that play in this expression as a set
  virtual std::set<Key> keys() const {
    std::set<Key> keys;
    return keys;
  }

  // Return size needed for memory buffer in traceExecution
  size_t traceSize() const {
    return traceSize_;
  }

  /// Return value
  virtual T value(const Values& values) const = 0;

  /// Return value and derivatives
  virtual Augmented<T> forward(const Values& values) const = 0;

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
// Below we use the "Class Composition" technique described in the book
//   C++ Template Metaprogramming: Concepts, Tools, and Techniques from Boost
//   and Beyond. Abrahams, David; Gurtovoy, Aleksey. Pearson Education.
// to recursively generate a class, that will be the base for function nodes.
// The class generated, for two arguments A1, A2, and A3 will be
//
// struct Base1 : Argument<T,A1,1>, FunctionalBase<T> {
//   ... storage related to A1 ...
//   ... methods that work on A1 ...
// };
//
// struct Base2 : Argument<T,A2,2>, Base1 {
//   ... storage related to A2 ...
//   ... methods that work on A2 and (recursively) on A2 ...
// };
//
// struct Base2 : Argument<T,A3,3>, Base2 {
//   ... storage related to A3 ...
//   ... methods that work on A3 and (recursively) on A2 and A3 ...
// };
//
// struct FunctionalNode : Base3 {
//   Provides convenience access to storage in hierarchy by using
//   static_cast<Argument<T, A, N> &>(*this)
// }
//
// All this magic happens when  we generate the Base3 base class of FunctionalNode
// by invoking boost::mpl::fold over the meta-function GenerateFunctionalNode
//-----------------------------------------------------------------------------

/// meta-function to generate fixed-size JacobianTA type
template<class T, class A>
struct Jacobian {
  typedef Eigen::Matrix<double, T::dimension, A::dimension> type;
  typedef boost::optional<type&> optional;
};

/**
 * Base case for recursive FunctionalNode class
 */
template<class T>
struct FunctionalBase: ExpressionNode<T> {
  static size_t const N = 0; // number of arguments

  typedef CallRecord<T::dimension> Record;

  /// Construct an execution trace for reverse AD
  void trace(const Values& values, Record* record, char*& raw) const {
  }
};

/**
 * Building block for recursive FunctionalNode class
 * The integer argument N is to guarantee a unique type signature,
 * so we are guaranteed to be able to extract their values by static cast.
 */
template<class T, class A, size_t N>
struct Argument {
  /// Expression that will generate value/derivatives for argument
  boost::shared_ptr<ExpressionNode<A> > expression;
};

/**
 * Building block for Recursive Record Class
 * Records the evaluation of a single argument in a functional expression
 */
template<class T, class A, size_t N>
struct JacobianTrace {
  A value;
  ExecutionTrace<A> trace;
  typename Jacobian<T, A>::type dTdA;
};

/**
 * Recursive Definition of Functional ExpressionNode
 */
template<class T, class A, class Base>
struct GenerateFunctionalNode: Argument<T, A, Base::N + 1>, Base {

  static size_t const N = Base::N + 1; ///< Number of arguments in hierarchy
  typedef Argument<T, A, N> This; ///< The storage we have direct access to

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    std::set<Key> keys = Base::keys();
    std::set<Key> myKeys = This::expression->keys();
    keys.insert(myKeys.begin(), myKeys.end());
    return keys;
  }

  /// Recursive Record Class for Functional Expressions
  struct Record: JacobianTrace<T, A, N>, Base::Record {

    typedef T return_type;
    typedef JacobianTrace<T, A, N> This;

    /// Print to std::cout
    virtual void print(const std::string& indent) const {
      Base::Record::print(indent);
      static const Eigen::IOFormat matlab(0, 1, " ", "; ", "", "", "[", "]");
      std::cout << This::dTdA.format(matlab) << std::endl;
      This::trace.print(indent);
    }

    /// Start the reverse AD process
    virtual void startReverseAD(JacobianMap& jacobians) const {
      Base::Record::startReverseAD(jacobians);
      Select<T::dimension, A>::reverseAD(This::trace, This::dTdA, jacobians);
    }

    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const {
      Base::Record::reverseAD(dFdT, jacobians);
      This::trace.reverseAD(dFdT * This::dTdA, jacobians);
    }

    /// Version specialized to 2-dimensional output
    typedef Eigen::Matrix<double, 2, T::dimension> Jacobian2T;
    virtual void reverseAD2(const Jacobian2T& dFdT,
        JacobianMap& jacobians) const {
      Base::Record::reverseAD2(dFdT, jacobians);
      This::trace.reverseAD2(dFdT * This::dTdA, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  void trace(const Values& values, Record* record, char*& raw) const {
    Base::trace(values, record, raw); // recurse
    // Write an Expression<A> execution trace in record->trace
    // Iff Constant or Leaf, this will not write to raw, only to trace.
    // Iff the expression is functional, write all Records in raw buffer
    // Return value of type T is recorded in record->value
    record->Record::This::value = This::expression->traceExecution(values,
        record->Record::This::trace, raw);
    // raw is never modified by traceExecution, but if traceExecution has
    // written in the buffer, the next caller expects we advance the pointer
    raw += This::expression->traceSize();
  }
};

/**
 *  Recursive GenerateFunctionalNode class Generator
 */
template<class T, class TYPES>
struct FunctionalNode {
  typedef typename boost::mpl::fold<TYPES, FunctionalBase<T>,
      GenerateFunctionalNode<T, MPL::_2, MPL::_1> >::type Base;

  struct type: public Base {

    /// Reset expression shared pointer
    template<class A, size_t N>
    void reset(const boost::shared_ptr<ExpressionNode<A> >& ptr) {
      static_cast<Argument<T, A, N> &>(*this).expression = ptr;
    }

    /// Access Expression shared pointer
    template<class A, size_t N>
    boost::shared_ptr<ExpressionNode<A> > expression() const {
      return static_cast<Argument<T, A, N> const &>(*this).expression;
    }

    /// Provide convenience access to Record storage
    struct Record: public Base::Record {

      /// Access Value
      template<class A, size_t N>
      const A& value() const {
        return static_cast<JacobianTrace<T, A, N> const &>(*this).value;
      }

      /// Access Jacobian
      template<class A, size_t N>
      typename Jacobian<T, A>::type& jacobian() {
        return static_cast<JacobianTrace<T, A, N>&>(*this).dTdA;
      }

    };

    /// Construct an execution trace for reverse AD
    Record* trace(const Values& values, char* raw) const {

      // Create the record and advance the pointer
      Record* record = new (raw) Record();
      raw = (char*) (record + 1);

      // Record the traces for all arguments
      // After this, the raw pointer is set to after what was written
      Base::trace(values, record, raw);

      // Return the record for this function evaluation
      return record;
    }
  };
};
//-----------------------------------------------------------------------------

/// Unary Function Expression
template<class T, class A1>
class UnaryExpression: public FunctionalNode<T, boost::mpl::vector<A1> >::type {

public:

  typedef boost::function<T(const A1&, typename Jacobian<T, A1>::optional)> Function;
  typedef typename FunctionalNode<T, boost::mpl::vector<A1> >::type Base;
  typedef typename Base::Record Record;

private:

  Function function_;

  /// Constructor with a unary function f, and input argument e
  UnaryExpression(Function f, const Expression<A1>& e1) :
      function_(f) {
    this->template reset<A1, 1>(e1.root());
    ExpressionNode<T>::traceSize_ = sizeof(Record) + e1.traceSize();
  }

  friend class Expression<T> ;

public:

  /// Return value
  virtual T value(const Values& values) const {
    return function_(this->template expression<A1, 1>()->value(values), boost::none);
  }

  /// Return value and derivatives
  virtual Augmented<T> forward(const Values& values) const {
    using boost::none;
    Augmented<A1> a1 = this->template expression<A1, 1>()->forward(values);
    typename Jacobian<T, A1>::type dTdA1;
    T t = function_(a1.value(),
        a1.constant() ? none : typename Jacobian<T,A1>::optional(dTdA1));
    return Augmented<T>(t, dTdA1, a1.jacobians());
  }

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const {

    Record* record = Base::trace(values, raw);
    trace.setFunction(record);

    return function_(record->template value<A1, 1>(),
        record->template jacobian<A1, 1>());
  }
};

//-----------------------------------------------------------------------------
/// Binary Expression

template<class T, class A1, class A2>
class BinaryExpression: public FunctionalNode<T, boost::mpl::vector<A1, A2> >::type {

public:

  typedef boost::function<
      T(const A1&, const A2&, typename Jacobian<T, A1>::optional,
          typename Jacobian<T, A2>::optional)> Function;
  typedef typename FunctionalNode<T, boost::mpl::vector<A1, A2> >::type Base;
  typedef typename Base::Record Record;

private:

  Function function_;

  /// Constructor with a ternary function f, and three input arguments
  BinaryExpression(Function f, const Expression<A1>& e1,
      const Expression<A2>& e2) :
      function_(f) {
    this->template reset<A1, 1>(e1.root());
    this->template reset<A2, 2>(e2.root());
    ExpressionNode<T>::traceSize_ = //
        sizeof(Record) + e1.traceSize() + e2.traceSize();
  }

  friend class Expression<T> ;
  friend class ::ExpressionFactorBinaryTest;

public:

  /// Return value
  virtual T value(const Values& values) const {
    using boost::none;
    return function_(this->template expression<A1, 1>()->value(values),
    this->template expression<A2, 2>()->value(values),
    none, none);
  }

  /// Return value and derivatives
  virtual Augmented<T> forward(const Values& values) const {
    using boost::none;
    Augmented<A1> a1 = this->template expression<A1, 1>()->forward(values);
    Augmented<A2> a2 = this->template expression<A2, 2>()->forward(values);
    typename Jacobian<T, A1>::type dTdA1;
    typename Jacobian<T, A2>::type dTdA2;
    T t = function_(a1.value(), a2.value(),
        a1.constant() ? none : typename Jacobian<T, A1>::optional(dTdA1),
        a2.constant() ? none : typename Jacobian<T, A2>::optional(dTdA2));
    return Augmented<T>(t, dTdA1, a1.jacobians(), dTdA2, a2.jacobians());
  }

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const {

    Record* record = Base::trace(values, raw);
    trace.setFunction(record);

    return function_(record->template value<A1, 1>(),
        record->template value<A2,2>(), record->template jacobian<A1, 1>(),
        record->template jacobian<A2, 2>());
  }
};

//-----------------------------------------------------------------------------
/// Ternary Expression

template<class T, class A1, class A2, class A3>
class TernaryExpression: public FunctionalNode<T, boost::mpl::vector<A1, A2, A3> >::type {

public:

  typedef boost::function<
      T(const A1&, const A2&, const A3&, typename Jacobian<T, A1>::optional,
          typename Jacobian<T, A2>::optional,
          typename Jacobian<T, A3>::optional)> Function;
  typedef typename FunctionalNode<T, boost::mpl::vector<A1, A2, A3> >::type Base;
  typedef typename Base::Record Record;

private:

  Function function_;

  /// Constructor with a ternary function f, and three input arguments
  TernaryExpression(Function f, const Expression<A1>& e1,
      const Expression<A2>& e2, const Expression<A3>& e3) :
      function_(f) {
    this->template reset<A1, 1>(e1.root());
    this->template reset<A2, 2>(e2.root());
    this->template reset<A3, 3>(e3.root());
    ExpressionNode<T>::traceSize_ = //
        sizeof(Record) + e1.traceSize() + e2.traceSize() + e3.traceSize();
  }

  friend class Expression<T> ;

public:

  /// Return value
  virtual T value(const Values& values) const {
    using boost::none;
    return function_(this->template expression<A1, 1>()->value(values),
    this->template expression<A2, 2>()->value(values),
    this->template expression<A3, 3>()->value(values),
    none, none, none);
  }

  /// Return value and derivatives
  virtual Augmented<T> forward(const Values& values) const {
    using boost::none;
    Augmented<A1> a1 = this->template expression<A1, 1>()->forward(values);
    Augmented<A2> a2 = this->template expression<A2, 2>()->forward(values);
    Augmented<A3> a3 = this->template expression<A3, 3>()->forward(values);
    typename Jacobian<T, A1>::type dTdA1;
    typename Jacobian<T, A2>::type dTdA2;
    typename Jacobian<T, A3>::type dTdA3;
    T t = function_(a1.value(), a2.value(), a3.value(),
        a1.constant() ? none : typename Jacobian<T, A1>::optional(dTdA1),
        a2.constant() ? none : typename Jacobian<T, A2>::optional(dTdA2),
        a3.constant() ? none : typename Jacobian<T, A3>::optional(dTdA3));
    return Augmented<T>(t, dTdA1, a1.jacobians(), dTdA2, a2.jacobians(), dTdA3,
        a3.jacobians());
  }

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const {

    Record* record = Base::trace(values, raw);
    trace.setFunction(record);

    return function_(
        record->template value<A1, 1>(), record->template value<A2, 2>(),
        record->template value<A3, 3>(), record->template jacobian<A1, 1>(),
        record->template jacobian<A2, 2>(), record->template jacobian<A3, 3>());
  }

};
//-----------------------------------------------------------------------------
}

