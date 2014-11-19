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
#include <gtsam/base/Manifold.h>
#include <gtsam/base/VerticalBlockMatrix.h>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/bind.hpp>

// template meta-programming headers
#include <boost/mpl/vector.hpp>
#include <boost/mpl/plus.hpp>
#include <boost/mpl/front.hpp>
#include <boost/mpl/pop_front.hpp>
#include <boost/mpl/fold.hpp>
#include <boost/mpl/empty_base.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/at.hpp>
namespace MPL = boost::mpl::placeholders;

#include <new> // for placement new
class ExpressionFactorBinaryTest;
// Forward declare for testing

namespace gtsam {

template<typename T>
class Expression;

/**
 * Expressions are designed to write their derivatives into an already allocated
 * Jacobian of the correct size, of type VerticalBlockMatrix.
 * The JacobianMap provides a mapping from keys to the underlying blocks.
 */
class JacobianMap {
  const FastVector<Key>& keys_;
  VerticalBlockMatrix& Ab_;
public:
  JacobianMap(const FastVector<Key>& keys, VerticalBlockMatrix& Ab) :
      keys_(keys), Ab_(Ab) {
  }
  /// Access via key
  VerticalBlockMatrix::Block operator()(Key key) {
    FastVector<Key>::const_iterator it = std::find(keys_.begin(), keys_.end(),
        key);
    DenseIndex block = it - keys_.begin();
    return Ab_(block);
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
/// Handle Leaf Case: reverseAD ends here, by writing a matrix into Jacobians
template<int ROWS, int COLS>
void handleLeafCase(const Eigen::Matrix<double, ROWS, COLS>& dTdA,
    JacobianMap& jacobians, Key key) {
  jacobians(key).block<ROWS, COLS>(0, 0) += dTdA; // block makes HUGE difference
}
/// Handle Leaf Case for Dynamic Matrix type (slower)
template<>
inline void handleLeafCase(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& dTdA,
    JacobianMap& jacobians, Key key) {
  jacobians(key) += dTdA;
}

//-----------------------------------------------------------------------------
/**
 * The ExecutionTrace class records a tree-structured expression's execution.
 *
 * The class looks a bit complicated but it is so for performance.
 * It is a tagged union that obviates the need to create
 * a ExecutionTrace subclass for Constants and Leaf Expressions. Instead
 * the key for the leaf is stored in the space normally used to store a
 * CallRecord*. Nothing is stored for a Constant.
 *
 * A full execution trace of a Binary(Unary(Binary(Leaf,Constant)),Leaf) would be:
 * Trace(Function) ->
 *   BinaryRecord with two traces in it
 *     trace1(Function) ->
 *       UnaryRecord with one trace in it
 *         trace1(Function) ->
 *           BinaryRecord with two traces in it
 *             trace1(Leaf)
 *             trace2(Constant)
 *     trace2(Leaf)
 * Hence, there are three Record structs, written to memory by traceExecution
 */
template<class T>
class ExecutionTrace {
  static const int Dim = traits::dimension<T>::value;
  enum {
    Constant, Leaf, Function
  } kind;
  union {
    Key key;
    CallRecord<Dim>* ptr;
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
  void setFunction(CallRecord<Dim>* record) {
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
  /**
   *  *** This is the main entry point for reverseAD, called from Expression ***
   * Called only once, either inserts I into Jacobians (Leaf) or starts AD (Function)
   */
  typedef Eigen::Matrix<double, Dim, Dim> JacobianTT;
  void startReverseAD(JacobianMap& jacobians) const {
    if (kind == Leaf) {
      // This branch will only be called on trivial Leaf expressions, i.e. Priors
      static const JacobianTT I = JacobianTT::Identity();
      handleLeafCase(I, jacobians, content.key);
    } else if (kind == Function)
      // This is the more typical entry point, starting the AD pipeline
      // Inside the startReverseAD that the correctly dimensioned pipeline is chosen.
      content.ptr->startReverseAD(jacobians);
  }
  // Either add to Jacobians (Leaf) or propagate (Function)
  void reverseAD(const Matrix& dTdA, JacobianMap& jacobians) const {
    if (kind == Leaf)
      handleLeafCase(dTdA, jacobians, content.key);
    else if (kind == Function)
      content.ptr->reverseAD(dTdA, jacobians);
  }
  // Either add to Jacobians (Leaf) or propagate (Function)
  typedef Eigen::Matrix<double, 2, Dim> Jacobian2T;
  void reverseAD2(const Jacobian2T& dTdA, JacobianMap& jacobians) const {
    if (kind == Leaf)
      handleLeafCase(dTdA, jacobians, content.key);
    else if (kind == Function)
      content.ptr->reverseAD2(dTdA, jacobians);
  }

  /// Define type so we can apply it as a meta-function
  typedef ExecutionTrace<T> type;
};

/// Primary template calls the generic Matrix reverseAD pipeline
template<size_t ROWS, class A>
struct Select {
  typedef Eigen::Matrix<double, ROWS, traits::dimension<A>::value> Jacobian;
  static void reverseAD(const ExecutionTrace<A>& trace, const Jacobian& dTdA,
      JacobianMap& jacobians) {
    trace.reverseAD(dTdA, jacobians);
  }
};

/// Partially specialized template calls the 2-dimensional output version
template<class A>
struct Select<2, A> {
  typedef Eigen::Matrix<double, 2, traits::dimension<A>::value> Jacobian;
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

  /// Return dimensions for each argument, as a map
  virtual void dims(std::map<Key, size_t>& map) const {
  }

  // Return size needed for memory buffer in traceExecution
  size_t traceSize() const {
    return traceSize_;
  }

  /// Return value
  virtual T value(const Values& values) const = 0;

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

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      char* raw) const {
    return constant_;
  }
};

//-----------------------------------------------------------------------------
/// Leaf Expression
template<class T, class Chart = DefaultChart<T> >
class LeafExpression: public ExpressionNode<T> {
  typedef ChartValue<T, Chart> value_type; // perhaps this can be something else like a std::pair<T,Chart> ??

  /// The key into values
  Key key_;

  /// Constructor with a single key
  LeafExpression(Key key) :
      key_(key) {
  }
  // todo: do we need a virtual destructor here too?

  friend class Expression<value_type> ;

public:

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    std::set<Key> keys;
    keys.insert(key_);
    return keys;
  }

  /// Return dimensions for each argument
  virtual void dims(std::map<Key, size_t>& map) const {
    // get dimension from the chart; only works for fixed dimension charts
    map[key_] = traits::dimension<Chart>::value;
  }

  /// Return value
  virtual const value_type& value(const Values& values) const {
    return dynamic_cast<const value_type&>(values.at(key_));
  }

  /// Construct an execution trace for reverse AD
  virtual const value_type& traceExecution(const Values& values,
      ExecutionTrace<value_type>& trace, char* raw) const {
    trace.setLeaf(key_);
    return dynamic_cast<const value_type&>(values.at(key_));
  }

};

//-----------------------------------------------------------------------------
/// Leaf Expression, if no chart is given, assume default chart and value_type is just the plain value
template<typename T>
class LeafExpression<T, DefaultChart<T> > : public ExpressionNode<T> {
  typedef T value_type;

  /// The key into values
  Key key_;

  /// Constructor with a single key
  LeafExpression(Key key) :
      key_(key) {
  }
  // todo: do we need a virtual destructor here too?

  friend class Expression<T> ;

public:

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    std::set<Key> keys;
    keys.insert(key_);
    return keys;
  }

  /// Return dimensions for each argument
  virtual void dims(std::map<Key, size_t>& map) const {
    map[key_] = traits::dimension<T>::value;
  }

  /// Return value
  virtual T value(const Values& values) const {
    return values.at<T>(key_);
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
//
// The class generated, for three arguments A1, A2, and A3 will be
//
// struct Base1 : Argument<T,A1,1>, FunctionalBase<T> {
//   ... storage related to A1 ...
//   ... methods that work on A1 ...
// };
//
// struct Base2 : Argument<T,A2,2>, Base1 {
//   ... storage related to A2 ...
//   ... methods that work on A2 and (recursively) on A1 ...
// };
//
// struct Base3 : Argument<T,A3,3>, Base2 {
//   ... storage related to A3 ...
//   ... methods that work on A3 and (recursively) on A2 and A1 ...
// };
//
// struct FunctionalNode : Base3 {
//   Provides convenience access to storage in hierarchy by using
//   static_cast<Argument<T, A, N> &>(*this)
// }
//
// All this magic happens when  we generate the Base3 base class of FunctionalNode
// by invoking boost::mpl::fold over the meta-function GenerateFunctionalNode
//
// Similarly, the inner Record struct will be
//
// struct Record1 : JacobianTrace<T,A1,1>, CallRecord<traits::dimension<T>::value> {
//   ... storage related to A1 ...
//   ... methods that work on A1 ...
// };
//
// struct Record2 : JacobianTrace<T,A2,2>, Record1 {
//   ... storage related to A2 ...
//   ... methods that work on A2 and (recursively) on A1 ...
// };
//
// struct Record3 : JacobianTrace<T,A3,3>, Record2 {
//   ... storage related to A3 ...
//   ... methods that work on A3 and (recursively) on A2 and A1 ...
// };
//
// struct Record : Record3 {
//   Provides convenience access to storage in hierarchy by using
//   static_cast<JacobianTrace<T, A, N> &>(*this)
// }
//

//-----------------------------------------------------------------------------

/// meta-function to generate fixed-size JacobianTA type
template<class T, class A>
struct Jacobian {
  typedef Eigen::Matrix<double, traits::dimension<T>::value,
      traits::dimension<A>::value> type;
};

/// meta-function to generate JacobianTA optional reference
template<class T, class A>
struct OptionalJacobian {
  typedef Eigen::Matrix<double, traits::dimension<T>::value,
      traits::dimension<A>::value> Jacobian;
  typedef boost::optional<Jacobian&> type;
};

/**
 * Base case for recursive FunctionalNode class
 */
template<class T>
struct FunctionalBase: ExpressionNode<T> {
  static size_t const N = 0; // number of arguments

  typedef CallRecord<traits::dimension<T>::value> Record;

  /// Construct an execution trace for reverse AD
  void trace(const Values& values, Record* record, char*& raw) const {
    // base case: does not do anything
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

  /// Return dimensions for each argument
  virtual void dims(std::map<Key, size_t>& map) const {
    Base::dims(map);
    This::expression->dims(map);
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
      Select<traits::dimension<T>::value, A>::reverseAD(This::trace, This::dTdA,
          jacobians);
    }

    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    virtual void reverseAD(const Matrix& dFdT, JacobianMap& jacobians) const {
      Base::Record::reverseAD(dFdT, jacobians);
      This::trace.reverseAD(dFdT * This::dTdA, jacobians);
    }

    /// Version specialized to 2-dimensional output
    typedef Eigen::Matrix<double, 2, traits::dimension<T>::value> Jacobian2T;
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

  /// The following typedef generates the recursively defined Base class
  typedef typename boost::mpl::fold<TYPES, FunctionalBase<T>,
      GenerateFunctionalNode<T, MPL::_2, MPL::_1> >::type Base;

  /**
   *  The type generated by this meta-function derives from Base
   *  and adds access functions as well as the crucial [trace] function
   */
  struct type: public Base {

    // Argument types and derived, note these are base 0 !
    // These are currently not used - useful for Phoenix in future
#ifdef EXPRESSIONS_PHOENIX
    typedef TYPES Arguments;
    typedef typename boost::mpl::transform<TYPES, Jacobian<T, MPL::_1> >::type Jacobians;
    typedef typename boost::mpl::transform<TYPES, OptionalJacobian<T, MPL::_1> >::type Optionals;
#endif

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

  typedef boost::function<T(const A1&, typename OptionalJacobian<T, A1>::type)> Function;
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
      T(const A1&, const A2&, typename OptionalJacobian<T, A1>::type,
          typename OptionalJacobian<T, A2>::type)> Function;
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
      T(const A1&, const A2&, const A3&, typename OptionalJacobian<T, A1>::type,
          typename OptionalJacobian<T, A2>::type,
          typename OptionalJacobian<T, A3>::type)> Function;
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
