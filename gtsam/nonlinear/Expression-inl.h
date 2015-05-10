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
#include <gtsam/nonlinear/CallRecord.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/Lie.h>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/bind.hpp>
#include <boost/type_traits/aligned_storage.hpp>

// template meta-programming headers
#include <boost/mpl/fold.hpp>
namespace MPL = boost::mpl::placeholders;

#include <typeinfo>       // operator typeid
#include <map>

class ExpressionFactorBinaryTest;
// Forward declare for testing

namespace gtsam {

//-----------------------------------------------------------------------------
// ExecutionTrace.h
//-----------------------------------------------------------------------------

template<typename T>
T & upAlign(T & value, unsigned requiredAlignment = TraceAlignment) {
  // right now only word sized types are supported.
  // Easy to extend if needed,
  //   by somehow inferring the unsigned integer of same size
  BOOST_STATIC_ASSERT(sizeof(T) == sizeof(size_t));
  size_t & uiValue = reinterpret_cast<size_t &>(value);
  size_t misAlignment = uiValue % requiredAlignment;
  if (misAlignment) {
    uiValue += requiredAlignment - misAlignment;
  }
  return value;
}
template<typename T>
T upAligned(T value, unsigned requiredAlignment = TraceAlignment) {
  return upAlign(value, requiredAlignment);
}

//-----------------------------------------------------------------------------

namespace internal {

template<bool UseBlock, typename Derived>
struct UseBlockIf {
  static void addToJacobian(const Eigen::MatrixBase<Derived>& dTdA,
      JacobianMap& jacobians, Key key) {
    // block makes HUGE difference
    jacobians(key).block<Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>(
        0, 0) += dTdA;
  }
  ;
};
/// Handle Leaf Case for Dynamic Matrix type (slower)
template<typename Derived>
struct UseBlockIf<false, Derived> {
  static void addToJacobian(const Eigen::MatrixBase<Derived>& dTdA,
      JacobianMap& jacobians, Key key) {
    jacobians(key) += dTdA;
  }
};
}

/// Handle Leaf Case: reverse AD ends here, by writing a matrix into Jacobians
template<typename Derived>
void handleLeafCase(const Eigen::MatrixBase<Derived>& dTdA,
    JacobianMap& jacobians, Key key) {
  internal::UseBlockIf<
      Derived::RowsAtCompileTime != Eigen::Dynamic
          && Derived::ColsAtCompileTime != Eigen::Dynamic, Derived>::addToJacobian(
      dTdA, jacobians, key);
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
  static const int Dim = traits<T>::dimension;
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
   *  *** This is the main entry point for reverse AD, called from Expression ***
   * Called only once, either inserts I into Jacobians (Leaf) or starts AD (Function)
   */
  typedef Eigen::Matrix<double, Dim, Dim> JacobianTT;
  void startReverseAD1(JacobianMap& jacobians) const {
    if (kind == Leaf) {
      // This branch will only be called on trivial Leaf expressions, i.e. Priors
      static const JacobianTT I = JacobianTT::Identity();
      handleLeafCase(I, jacobians, content.key);
    } else if (kind == Function)
      // This is the more typical entry point, starting the AD pipeline
      // Inside startReverseAD2 the correctly dimensioned pipeline is chosen.
      content.ptr->startReverseAD2(jacobians);
  }
  // Either add to Jacobians (Leaf) or propagate (Function)
  template<typename DerivedMatrix>
  void reverseAD1(const Eigen::MatrixBase<DerivedMatrix> & dTdA,
      JacobianMap& jacobians) const {
    if (kind == Leaf)
      handleLeafCase(dTdA, jacobians, content.key);
    else if (kind == Function)
      content.ptr->reverseAD2(dTdA, jacobians);
  }

  /// Define type so we can apply it as a meta-function
  typedef ExecutionTrace<T> type;
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
      ExecutionTraceStorage* traceStorage) const {
    return constant_;
  }
};

//-----------------------------------------------------------------------------
/// Leaf Expression, if no chart is given, assume default chart and value_type is just the plain value
template<typename T>
class LeafExpression: public ExpressionNode<T> {
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
  virtual void dims(std::map<Key, int>& map) const {
    map[key_] = traits<T>::dimension;
  }

  /// Return value
  virtual T value(const Values& values) const {
    return values.at<T>(key_);
  }

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      ExecutionTraceStorage* traceStorage) const {
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
  typedef Eigen::Matrix<double, traits<T>::dimension, traits<A>::dimension> type;
};

/**
 * Base case for recursive FunctionalNode class
 */
template<class T>
struct FunctionalBase: ExpressionNode<T> {
  static size_t const N = 0; // number of arguments

  struct Record {
    void print(const std::string& indent) const {
    }
    void startReverseAD4(JacobianMap& jacobians) const {
    }
    template<typename SomeMatrix>
    void reverseAD4(const SomeMatrix & dFdT, JacobianMap& jacobians) const {
    }
  };
  /// Construct an execution trace for reverse AD
  void trace(const Values& values, Record* record,
      ExecutionTraceStorage*& traceStorage) const {
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

// Recursive Definition of Functional ExpressionNode
// The reason we inherit from Argument<T, A, N> is because we can then
// case to this unique signature to retrieve the expression at any level
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
  virtual void dims(std::map<Key, int>& map) const {
    Base::dims(map);
    This::expression->dims(map);
  }

  // Recursive Record Class for Functional Expressions
  // The reason we inherit from JacobianTrace<T, A, N> is because we can then
  // case to this unique signature to retrieve the value/trace at any level
  struct Record: JacobianTrace<T, A, N>, Base::Record {

    typedef T return_type;
    typedef JacobianTrace<T, A, N> This;

    /// Print to std::cout
    void print(const std::string& indent) const {
      Base::Record::print(indent);
      static const Eigen::IOFormat matlab(0, 1, " ", "; ", "", "", "[", "]");
      std::cout << This::dTdA.format(matlab) << std::endl;
      This::trace.print(indent);
    }

    /// Start the reverse AD process
    void startReverseAD4(JacobianMap& jacobians) const {
      Base::Record::startReverseAD4(jacobians);
      // This is the crucial point where the size of the AD pipeline is selected.
      // One pipeline is started for each argument, but the number of rows in each
      // pipeline is the same, namely the dimension of the output argument T.
      // For example, if the entire expression is rooted by a binary function
      // yielding a 2D result, then the matrix dTdA will have 2 rows.
      // ExecutionTrace::reverseAD1 just passes this on to CallRecord::reverseAD2
      // which calls the correctly sized CallRecord::reverseAD3, which in turn
      // calls reverseAD4 below.
      This::trace.reverseAD1(This::dTdA, jacobians);
    }

    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    // Cols is always known at compile time
    template<typename SomeMatrix>
    void reverseAD4(const SomeMatrix & dFdT, JacobianMap& jacobians) const {
      Base::Record::reverseAD4(dFdT, jacobians);
      This::trace.reverseAD1(dFdT * This::dTdA, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  void trace(const Values& values, Record* record,
      ExecutionTraceStorage*& traceStorage) const {
    Base::trace(values, record, traceStorage); // recurse
    // Write an Expression<A> execution trace in record->trace
    // Iff Constant or Leaf, this will not write to traceStorage, only to trace.
    // Iff the expression is functional, write all Records in traceStorage buffer
    // Return value of type T is recorded in record->value
    record->Record::This::value = This::expression->traceExecution(values,
        record->Record::This::trace, traceStorage);
    // traceStorage is never modified by traceExecution, but if traceExecution has
    // written in the buffer, the next caller expects we advance the pointer
    traceStorage += This::expression->traceSize();
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

    /// Provide convenience access to Record storage and implement
    /// the virtual function based interface of CallRecord using the CallRecordImplementor
    struct Record: public internal::CallRecordImplementor<Record,
        traits<T>::dimension>, public Base::Record {
      using Base::Record::print;
      using Base::Record::startReverseAD4;
      using Base::Record::reverseAD4;

      virtual ~Record() {
      }

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
    Record* trace(const Values& values,
        ExecutionTraceStorage* traceStorage) const {
      assert(reinterpret_cast<size_t>(traceStorage) % TraceAlignment == 0);

      // Create the record and advance the pointer
      Record* record = new (traceStorage) Record();
      traceStorage += upAligned(sizeof(Record));

      // Record the traces for all arguments
      // After this, the traceStorage pointer is set to after what was written
      Base::trace(values, record, traceStorage);

      // Return the record for this function evaluation
      return record;
    }
  };
};
//-----------------------------------------------------------------------------

/// Unary Function Expression
template<class T, class A1>
class UnaryExpression: public ExpressionNode<T> {

  typedef typename UnaryFunction<T,A1>::type Function;
  Function function_;
  boost::shared_ptr<ExpressionNode<A1> > expression1_;

  typedef Argument<T, A1, 1> This; ///< The storage we have direct access to

  /// Constructor with a unary function f, and input argument e
  UnaryExpression(Function f, const Expression<A1>& e1) :
      function_(f) {
    this->expression1_ = e1.root();
    ExpressionNode<T>::traceSize_ = upAligned(sizeof(Record)) + e1.traceSize();
  }

  friend class Expression<T> ;

public:

  /// Return value
  virtual T value(const Values& values) const {
    return function_(this->expression1_->value(values), boost::none);
  }

  /// Return keys that play in this expression
  virtual std::set<Key> keys() const {
    std::set<Key> keys; // = Base::keys();
    std::set<Key> myKeys = this->expression1_->keys();
    keys.insert(myKeys.begin(), myKeys.end());
    return keys;
  }

  /// Return dimensions for each argument
  virtual void dims(std::map<Key, int>& map) const {
    // Base::dims(map);
    this->expression1_->dims(map);
  }

  // Inner Record Class
  // The reason we inherit from JacobianTrace<T, A, N> is because we can then
  // case to this unique signature to retrieve the value/trace at any level
  struct Record: public internal::CallRecordImplementor<Record,
      traits<T>::dimension>, JacobianTrace<T, A1, 1> {

    typedef T return_type;
    typedef JacobianTrace<T, A1, 1> This;

    /// Access Jacobian
    template<class A, size_t N>
    typename Jacobian<T, A1>::type& jacobian() {
      return static_cast<JacobianTrace<T, A, N>&>(*this).dTdA;
    }

    /// Access Value
    template<class A, size_t N>
    const A& value() const {
      return static_cast<JacobianTrace<T, A, N> const &>(*this).value;
    }

    /// Print to std::cout
    void print(const std::string& indent) const {
      std::cout << indent << "UnaryExpression::Record {" << std::endl;
      static const Eigen::IOFormat matlab(0, 1, " ", "; ", "", "", "[", "]");
      std::cout << indent << This::dTdA.format(matlab) << std::endl;
      This::trace.print(indent);
      std::cout << indent << "}" << std::endl;
    }

    /// Start the reverse AD process
    void startReverseAD4(JacobianMap& jacobians) const {
      // This is the crucial point where the size of the AD pipeline is selected.
      // One pipeline is started for each argument, but the number of rows in each
      // pipeline is the same, namely the dimension of the output argument T.
      // For example, if the entire expression is rooted by a binary function
      // yielding a 2D result, then the matrix dTdA will have 2 rows.
      // ExecutionTrace::reverseAD1 just passes this on to CallRecord::reverseAD2
      // which calls the correctly sized CallRecord::reverseAD3, which in turn
      // calls reverseAD4 below.
      This::trace.reverseAD1(This::dTdA, jacobians);
    }

    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    // Cols is always known at compile time
    template<typename SomeMatrix>
    void reverseAD4(const SomeMatrix & dFdT, JacobianMap& jacobians) const {
      This::trace.reverseAD1(dFdT * This::dTdA, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  void trace(const Values& values, Record* record,
      ExecutionTraceStorage*& traceStorage) const {
    // Write an Expression<A> execution trace in record->trace
    // Iff Constant or Leaf, this will not write to traceStorage, only to trace.
    // Iff the expression is functional, write all Records in traceStorage buffer
    // Return value of type T is recorded in record->value
    record->Record::This::value = this->expression1_->traceExecution(values,
        record->Record::This::trace, traceStorage);
    // traceStorage is never modified by traceExecution, but if traceExecution has
    // written in the buffer, the next caller expects we advance the pointer
    traceStorage += this->expression1_->traceSize();
  }

  /// Construct an execution trace for reverse AD
  Record* trace(const Values& values,
      ExecutionTraceStorage* traceStorage) const {
    assert(reinterpret_cast<size_t>(traceStorage) % TraceAlignment == 0);

    // Create the record and advance the pointer
    Record* record = new (traceStorage) Record();
    traceStorage += upAligned(sizeof(Record));

    // Record the traces for all arguments
    // After this, the traceStorage pointer is set to after what was written
    this->trace(values, record, traceStorage);

    // Return the record for this function evaluation
    return record;
  }

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      ExecutionTraceStorage* traceStorage) const {

    Record* record = this->trace(values, traceStorage);
    trace.setFunction(record);

    return function_(record->template value<A1, 1>(),
        record->template jacobian<A1, 1>());
  }
};

//-----------------------------------------------------------------------------
/// Binary Expression

template<class T, class A1, class A2>
class BinaryExpression: public FunctionalNode<T, boost::mpl::vector<A1, A2> >::type {
  typedef typename FunctionalNode<T, boost::mpl::vector<A1, A2> >::type Base;

public:
  typedef typename Base::Record Record;

private:

  typedef typename BinaryFunction<T,A1,A2>::type Function;
  Function function_;

  /// Constructor with a ternary function f, and three input arguments
  BinaryExpression(Function f, const Expression<A1>& e1,
      const Expression<A2>& e2) :
      function_(f) {
    this->template reset<A1, 1>(e1.root());
    this->template reset<A2, 2>(e2.root());
    ExpressionNode<T>::traceSize_ = //
        upAligned(sizeof(Record)) + e1.traceSize() + e2.traceSize();
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
  ExecutionTraceStorage* traceStorage) const {

    Record* record = Base::trace(values, traceStorage);
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

  typedef typename FunctionalNode<T, boost::mpl::vector<A1, A2, A3> >::type Base;
  typedef typename Base::Record Record;

private:

  typedef typename TernaryFunction<T,A1,A2,A3>::type Function;
  Function function_;

  /// Constructor with a ternary function f, and three input arguments
  TernaryExpression(Function f, const Expression<A1>& e1,
      const Expression<A2>& e2, const Expression<A3>& e3) :
      function_(f) {
    this->template reset<A1, 1>(e1.root());
    this->template reset<A2, 2>(e2.root());
    this->template reset<A3, 3>(e3.root());
    ExpressionNode<T>::traceSize_ = //
        upAligned(sizeof(Record)) + e1.traceSize() + e2.traceSize()
            + e3.traceSize();
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
  ExecutionTraceStorage* traceStorage) const {

    Record* record = Base::trace(values, traceStorage);
    trace.setFunction(record);

    return function_(
    record->template value<A1, 1>(), record->template value<A2, 2>(),
    record->template value<A3, 3>(), record->template jacobian<A1, 1>(),
    record->template jacobian<A2, 2>(), record->template jacobian<A3, 3>());
  }

};

//-----------------------------------------------------------------------------
// Esxpression-inl.h
//-----------------------------------------------------------------------------

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
Expression<T>::Expression(typename UnaryFunction<T, A>::type function,
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
Expression<T>::Expression(typename BinaryFunction<T, A1, A2>::type function,
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
Expression<T>::Expression(
    typename TernaryFunction<T, A1, A2, A3>::type function,
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
T Expression<T>::value(const Values& values, boost::optional<std::vector<Matrix>&> H) const {

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
    ExecutionTraceStorage* traceStorage) const {
  return root_->traceExecution(values, trace, traceStorage);
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

// JacobianMap:
JacobianMap::JacobianMap(const FastVector<Key>& keys, VerticalBlockMatrix& Ab) :
    keys_(keys), Ab_(Ab) {
}

VerticalBlockMatrix::Block JacobianMap::operator()(Key key) {
  FastVector<Key>::const_iterator it = std::find(keys_.begin(), keys_.end(),
      key);
  DenseIndex block = it - keys_.begin();
  return Ab_(block);
}

//-----------------------------------------------------------------------------

}
