/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ExpressionNode.h
 * @date May 10, 2015
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief ExpressionNode class
 */

#pragma once

#include <gtsam/nonlinear/internal/ExecutionTrace.h>
#include <gtsam/nonlinear/internal/CallRecord.h>
#include <gtsam/nonlinear/Values.h>

#include <typeinfo>       // operator typeid
#include <ostream>
#include <map>

class ExpressionFactorBinaryTest;
// Forward declare for testing

namespace gtsam {
namespace internal {

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

/**
 * Expression node. The superclass for objects that do the heavy lifting
 * An Expression<T> has a pointer to an ExpressionNode<T> underneath
 * allowing Expressions to have polymorphic behaviour even though they
 * are passed by value. This is the same way std::function works.
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

  /// Print
  virtual void print(const std::string& indent = "") const = 0;

  /// Streaming
  GTSAM_EXPORT
  friend std::ostream& operator<<(std::ostream& os, const ExpressionNode& node) {
    os << "Expression of type " << demangle(typeid(T).name());
    if (node.traceSize_ > 0) os << ", trace size = " << node.traceSize_;
    os << "\n";
    return os;
  }

  /// Return keys that play in this expression as a set
  virtual std::set<Key> keys() const {
    std::set<Key> keys;
    return keys;
  }

  /// Return dimensions for each argument, as a map
  virtual void dims(std::map<Key, int>& map) const {
  }

  // Return size needed for memory buffer in traceExecution
  size_t traceSize() const {
    return traceSize_;
  }

  /// Return value
  virtual T value(const Values& values) const = 0;

  /// Construct an execution trace for reverse AD
  virtual T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      ExecutionTraceStorage* traceStorage) const = 0;
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
  ~ConstantExpression() override {
  }

  /// Print
  void print(const std::string& indent = "") const override {
    std::cout << indent << "Constant" << std::endl;
  }

  /// Return value
  T value(const Values& values) const override {
    return constant_;
  }

  /// Construct an execution trace for reverse AD
  T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      ExecutionTraceStorage* traceStorage) const override {
    return constant_;
  }

  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
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

  friend class Expression<T>;

public:

  /// Destructor
  ~LeafExpression() override {
  }

  /// Print
  void print(const std::string& indent = "") const override {
    std::cout << indent << "Leaf, key = " << DefaultKeyFormatter(key_) << std::endl;
  }

  /// Return keys that play in this expression
  std::set<Key> keys() const override {
    std::set<Key> keys;
    keys.insert(key_);
    return keys;
  }

  /// Return dimensions for each argument
  void dims(std::map<Key, int>& map) const override {
    map[key_] = traits<T>::dimension;
  }

  /// Return value
  T value(const Values& values) const override {
    return values.at<T>(key_);
  }

  /// Construct an execution trace for reverse AD
  T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      ExecutionTraceStorage* traceStorage) const override {
    trace.setLeaf(key_);
    return values.at<T>(key_);
  }

};

//-----------------------------------------------------------------------------
/// meta-function to generate fixed-size JacobianTA type
template<class T, class A>
struct Jacobian {
  typedef Eigen::Matrix<double, traits<T>::dimension, traits<A>::dimension> type;
};

// Helper function for printing Jacobians with compact Eigen format, and trace
template <class T, class A>
static void PrintJacobianAndTrace(const std::string& indent,
                                  const typename Jacobian<T, A>::type& dTdA,
                                  const ExecutionTrace<A> trace) {
  static const Eigen::IOFormat kMatlabFormat(0, 1, " ", "; ", "", "", "[", "]");
  std::cout << indent << "D(" << demangle(typeid(T).name()) << ")/D(" << demangle(typeid(A).name())
            << ") = " << dTdA.format(kMatlabFormat) << std::endl;
  trace.print(indent);
}

//-----------------------------------------------------------------------------
/// Unary Function Expression
template<class T, class A1>
class UnaryExpression: public ExpressionNode<T> {

  typedef typename Expression<T>::template UnaryFunction<A1>::type Function;
  boost::shared_ptr<ExpressionNode<A1> > expression1_;
  Function function_;

  /// Constructor with a unary function f, and input argument e1
  UnaryExpression(Function f, const Expression<A1>& e1) :
      expression1_(e1.root()), function_(f) {
    this->traceSize_ = upAligned(sizeof(Record)) + e1.traceSize();
  }

  friend class Expression<T>;

public:

  /// Destructor
  ~UnaryExpression() override {
  }

  /// Print
  void print(const std::string& indent = "") const override {
    std::cout << indent << "UnaryExpression" << std::endl;
    expression1_->print(indent + "  ");
  }

  /// Return value
  T value(const Values& values) const override {
    return function_(expression1_->value(values), {});
  }

  /// Return keys that play in this expression
  std::set<Key> keys() const override {
    return expression1_->keys();
  }

  /// Return dimensions for each argument
  void dims(std::map<Key, int>& map) const override {
    expression1_->dims(map);
  }

  // Inner Record Class
  struct Record: public CallRecordImplementor<Record, traits<T>::dimension> {

    typename Jacobian<T, A1>::type dTdA1;
    ExecutionTrace<A1> trace1;
    A1 value1;

    /// Construct record by calling argument expression
    Record(const Values& values, const ExpressionNode<A1>& expression1, ExecutionTraceStorage* ptr)
        : value1(expression1.traceExecution(values, trace1, ptr + upAligned(sizeof(Record)))) {}

    /// Print to std::cout
    void print(const std::string& indent) const {
      std::cout << indent << "UnaryExpression::Record {" << std::endl;
      PrintJacobianAndTrace<T,A1>(indent, dTdA1, trace1);
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
      trace1.reverseAD1(dTdA1, jacobians);
    }

    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    template<typename MatrixType>
    void reverseAD4(const MatrixType & dFdT, JacobianMap& jacobians) const {
      trace1.reverseAD1(dFdT * dTdA1, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      ExecutionTraceStorage* ptr) const override {
    assert(reinterpret_cast<size_t>(ptr) % TraceAlignment == 0);

    // Create a Record in the memory pointed to by ptr
    // Calling the constructor will record the traces for all arguments
    // Write an Expression<A> execution trace in record->trace
    // Iff Constant or Leaf, this will not write to traceStorage, only to trace.
    // Iff the expression is functional, write all Records in traceStorage buffer
    // Return value of type T is recorded in record->value
    // NOTE(frank, abe): The destructor on this record is never called due to this placement new
    // Records must only contain statically sized objects!
    Record* record = new (ptr) Record(values, *expression1_, ptr);

    // Our trace parameter is set to point to the Record
    trace.setFunction(record);

    // Finally, the function call fills in the Jacobian dTdA1
    return function_(record->value1, record->dTdA1);
  }
};

//-----------------------------------------------------------------------------
/// Binary Expression
template<class T, class A1, class A2>
class BinaryExpression: public ExpressionNode<T> {

  typedef typename Expression<T>::template BinaryFunction<A1, A2>::type Function;
  boost::shared_ptr<ExpressionNode<A1> > expression1_;
  boost::shared_ptr<ExpressionNode<A2> > expression2_;
  Function function_;

  /// Constructor with a binary function f, and two input arguments
  BinaryExpression(Function f, const Expression<A1>& e1,
      const Expression<A2>& e2) :
      expression1_(e1.root()), expression2_(e2.root()), function_(f) {
    this->traceSize_ = //
        upAligned(sizeof(Record)) + e1.traceSize() + e2.traceSize();
  }

  friend class Expression<T>;
  friend class ::ExpressionFactorBinaryTest;

public:

  /// Destructor
  ~BinaryExpression() override {
  }

  /// Print
  void print(const std::string& indent = "") const override {
    std::cout << indent << "BinaryExpression" << std::endl;
    expression1_->print(indent + "  ");
    expression2_->print(indent + "  ");
  }

  /// Return value
  T value(const Values& values) const override {
    using std::nullopt;
    return function_(expression1_->value(values), expression2_->value(values),
        {}, {});
  }

  /// Return keys that play in this expression
  std::set<Key> keys() const override {
    std::set<Key> keys = expression1_->keys();
    std::set<Key> myKeys = expression2_->keys();
    keys.insert(myKeys.begin(), myKeys.end());
    return keys;
  }

  /// Return dimensions for each argument
  void dims(std::map<Key, int>& map) const override {
    expression1_->dims(map);
    expression2_->dims(map);
  }

  // Inner Record Class
  struct Record: public CallRecordImplementor<Record, traits<T>::dimension> {

    typename Jacobian<T, A1>::type dTdA1;
    typename Jacobian<T, A2>::type dTdA2;

    ExecutionTrace<A1> trace1;
    ExecutionTrace<A2> trace2;

    // TODO(frank): These aren't needed kill them!
    A1 value1;
    A2 value2;

    /// Construct record by calling argument expressions
    Record(const Values& values, const ExpressionNode<A1>& expression1,
           const ExpressionNode<A2>& expression2, ExecutionTraceStorage* ptr)
        : value1(expression1.traceExecution(values, trace1, ptr += upAligned(sizeof(Record)))),
          value2(expression2.traceExecution(values, trace2, ptr += expression1.traceSize())) {}

    /// Print to std::cout
    void print(const std::string& indent) const {
      std::cout << indent << "BinaryExpression::Record {" << std::endl;
      PrintJacobianAndTrace<T,A1>(indent, dTdA1, trace1);
      PrintJacobianAndTrace<T,A2>(indent, dTdA2, trace2);
      std::cout << indent << "}" << std::endl;
    }

    /// Start the reverse AD process, see comments in UnaryExpression
    void startReverseAD4(JacobianMap& jacobians) const {
      trace1.reverseAD1(dTdA1, jacobians);
      trace2.reverseAD1(dTdA2, jacobians);
    }

    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    template<typename MatrixType>
    void reverseAD4(const MatrixType & dFdT, JacobianMap& jacobians) const {
      trace1.reverseAD1(dFdT * dTdA1, jacobians);
      trace2.reverseAD1(dFdT * dTdA2, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD, see UnaryExpression for explanation
  T traceExecution(const Values& values, ExecutionTrace<T>& trace,
      ExecutionTraceStorage* ptr) const override {
    assert(reinterpret_cast<size_t>(ptr) % TraceAlignment == 0);
    Record* record = new (ptr) Record(values, *expression1_, *expression2_, ptr);
    trace.setFunction(record);
    return function_(record->value1, record->value2, record->dTdA1, record->dTdA2);
  }
};

//-----------------------------------------------------------------------------
/// Ternary Expression
template<class T, class A1, class A2, class A3>
class TernaryExpression: public ExpressionNode<T> {

  typedef typename Expression<T>::template TernaryFunction<A1, A2, A3>::type Function;
  boost::shared_ptr<ExpressionNode<A1> > expression1_;
  boost::shared_ptr<ExpressionNode<A2> > expression2_;
  boost::shared_ptr<ExpressionNode<A3> > expression3_;
  Function function_;

  /// Constructor with a ternary function f, and two input arguments
  TernaryExpression(Function f, const Expression<A1>& e1,
      const Expression<A2>& e2, const Expression<A3>& e3) :
      expression1_(e1.root()), expression2_(e2.root()), expression3_(e3.root()), //
      function_(f) {
    this->traceSize_ = upAligned(sizeof(Record)) + //
        e1.traceSize() + e2.traceSize() + e3.traceSize();
  }

  friend class Expression<T>;

public:

  /// Destructor
  ~TernaryExpression() override {
  }

  /// Print
  void print(const std::string& indent = "") const override {
    std::cout << indent << "TernaryExpression" << std::endl;
    expression1_->print(indent + "  ");
    expression2_->print(indent + "  ");
    expression3_->print(indent + "  ");
  }

  /// Return value
  T value(const Values& values) const override {
    using std::nullopt;
    return function_(expression1_->value(values), expression2_->value(values),
        expression3_->value(values), {}, {}, {});
  }

  /// Return keys that play in this expression
  std::set<Key> keys() const override {
    std::set<Key> keys = expression1_->keys();
    std::set<Key> myKeys = expression2_->keys();
    keys.insert(myKeys.begin(), myKeys.end());
    myKeys = expression3_->keys();
    keys.insert(myKeys.begin(), myKeys.end());
    return keys;
  }

  /// Return dimensions for each argument
  void dims(std::map<Key, int>& map) const override {
    expression1_->dims(map);
    expression2_->dims(map);
    expression3_->dims(map);
  }

  // Inner Record Class
  struct Record: public CallRecordImplementor<Record, traits<T>::dimension> {

    typename Jacobian<T, A1>::type dTdA1;
    typename Jacobian<T, A2>::type dTdA2;
    typename Jacobian<T, A3>::type dTdA3;

    ExecutionTrace<A1> trace1;
    ExecutionTrace<A2> trace2;
    ExecutionTrace<A3> trace3;

    A1 value1;
    A2 value2;
    A3 value3;

    /// Construct record by calling 3 argument expressions
    Record(const Values& values, const ExpressionNode<A1>& expression1,
           const ExpressionNode<A2>& expression2,
           const ExpressionNode<A3>& expression3, ExecutionTraceStorage* ptr)
        : value1(expression1.traceExecution(values, trace1, ptr += upAligned(sizeof(Record)))),
          value2(expression2.traceExecution(values, trace2, ptr += expression1.traceSize())),
          value3(expression3.traceExecution(values, trace3, ptr += expression2.traceSize())) {}

    /// Print to std::cout
    void print(const std::string& indent) const {
      std::cout << indent << "TernaryExpression::Record {" << std::endl;
      PrintJacobianAndTrace<T,A1>(indent, dTdA1, trace1);
      PrintJacobianAndTrace<T,A2>(indent, dTdA2, trace2);
      PrintJacobianAndTrace<T,A3>(indent, dTdA3, trace3);
      std::cout << indent << "}" << std::endl;
    }

    /// Start the reverse AD process, see comments in Base
    void startReverseAD4(JacobianMap& jacobians) const {
      trace1.reverseAD1(dTdA1, jacobians);
      trace2.reverseAD1(dTdA2, jacobians);
      trace3.reverseAD1(dTdA3, jacobians);
    }

    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    template<typename MatrixType>
    void reverseAD4(const MatrixType & dFdT, JacobianMap& jacobians) const {
      trace1.reverseAD1(dFdT * dTdA1, jacobians);
      trace2.reverseAD1(dFdT * dTdA2, jacobians);
      trace3.reverseAD1(dFdT * dTdA3, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD, see UnaryExpression for explanation
  T traceExecution(const Values& values, ExecutionTrace<T>& trace,
                           ExecutionTraceStorage* ptr) const override {
    assert(reinterpret_cast<size_t>(ptr) % TraceAlignment == 0);
    Record* record = new (ptr) Record(values, *expression1_, *expression2_, *expression3_, ptr);
    trace.setFunction(record);
    return function_(record->value1, record->value2, record->value3,
                     record->dTdA1, record->dTdA2, record->dTdA3);
  }
};

//-----------------------------------------------------------------------------
/// Expression for scalar multiplication
template <class T>
class ScalarMultiplyNode : public ExpressionNode<T> {
  // Check that T is a vector space
  BOOST_CONCEPT_ASSERT((gtsam::IsVectorSpace<T>));

  double scalar_;
  boost::shared_ptr<ExpressionNode<T> > expression_;

 public:
  /// Constructor with a unary function f, and input argument e1
  ScalarMultiplyNode(double s, const Expression<T>& e) : scalar_(s), expression_(e.root()) {
    this->traceSize_ = upAligned(sizeof(Record)) + e.traceSize();
  }

  /// Destructor
  ~ScalarMultiplyNode() override {}

  /// Print
  void print(const std::string& indent = "") const override {
    std::cout << indent << "ScalarMultiplyNode" << std::endl;
    expression_->print(indent + "  ");
  }

  /// Return value
  T value(const Values& values) const override {
    return scalar_ * expression_->value(values);
  }

  /// Return keys that play in this expression
  std::set<Key> keys() const override {
    return expression_->keys();
  }

  /// Return dimensions for each argument
  void dims(std::map<Key, int>& map) const override {
    expression_->dims(map);
  }

  // Inner Record Class
  struct Record : public CallRecordImplementor<Record, traits<T>::dimension> {
    static const int Dim = traits<T>::dimension;
    typedef Eigen::Matrix<double, Dim, Dim> JacobianTT;

    double scalar_dTdA;
    ExecutionTrace<T> trace;

    /// Print to std::cout
    void print(const std::string& indent) const {
      std::cout << indent << "ScalarMultiplyNode::Record {" << std::endl;
      std::cout << indent << "D(" << demangle(typeid(T).name()) << ")/D(" << demangle(typeid(T).name())
                << ") = " << scalar_dTdA << std::endl;
      trace.print();
      std::cout << indent << "}" << std::endl;
    }

    /// Start the reverse AD process
    void startReverseAD4(JacobianMap& jacobians) const {
      trace.reverseAD1(scalar_dTdA * JacobianTT::Identity(), jacobians);
    }

    /// Given df/dT, multiply in dT/dA and continue reverse AD process
    template <typename MatrixType>
    void reverseAD4(const MatrixType& dFdT, JacobianMap& jacobians) const {
      trace.reverseAD1(dFdT * scalar_dTdA, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  T traceExecution(const Values& values, ExecutionTrace<T>& trace,
                           ExecutionTraceStorage* ptr) const override {
    assert(reinterpret_cast<size_t>(ptr) % TraceAlignment == 0);
    Record* record = new (ptr) Record();
    ptr += upAligned(sizeof(Record));
    T value = expression_->traceExecution(values, record->trace, ptr);
    ptr += expression_->traceSize();
    trace.setFunction(record);
    record->scalar_dTdA = scalar_;
    return scalar_ * value;
  }
};


//-----------------------------------------------------------------------------
/// Binary Sum Expression
template <class T>
class BinarySumNode : public ExpressionNode<T> {
  typedef ExpressionNode<T> NodeT;
  boost::shared_ptr<ExpressionNode<T> > expression1_;
  boost::shared_ptr<ExpressionNode<T> > expression2_;

 public:
  explicit BinarySumNode() {
    this->traceSize_ = upAligned(sizeof(Record));
  }

  /// Constructor with a binary function f, and two input arguments
  BinarySumNode(const Expression<T>& e1, const Expression<T>& e2)
      : expression1_(e1.root()), expression2_(e2.root()) {
    this->traceSize_ =  //
        upAligned(sizeof(Record)) + e1.traceSize() + e2.traceSize();
  }

  /// Destructor
  ~BinarySumNode() override {}

  /// Print
  void print(const std::string& indent = "") const override {
    std::cout << indent << "BinarySumNode" << std::endl;
    expression1_->print(indent + "  ");
    expression2_->print(indent + "  ");
  }

  /// Return value
  T value(const Values& values) const override {
    return expression1_->value(values) + expression2_->value(values);
  }

  /// Return keys that play in this expression
  std::set<Key> keys() const override {
    std::set<Key> keys = expression1_->keys();
    std::set<Key> myKeys = expression2_->keys();
    keys.insert(myKeys.begin(), myKeys.end());
    return keys;
  }

  /// Return dimensions for each argument
  void dims(std::map<Key, int>& map) const override {
    expression1_->dims(map);
    expression2_->dims(map);
  }

  // Inner Record Class
  struct Record : public CallRecordImplementor<Record, traits<T>::dimension> {
    ExecutionTrace<T> trace1;
    ExecutionTrace<T> trace2;

    /// Print to std::cout
    void print(const std::string& indent) const {
      std::cout << indent << "BinarySumNode::Record {" << std::endl;
      trace1.print(indent);
      trace2.print(indent);
      std::cout << indent << "}" << std::endl;
    }

    /// If the BinarySumExpression is the root, we just start as many pipelines as there are terms.
    void startReverseAD4(JacobianMap& jacobians) const {
      // NOTE(frank): equivalent to trace.reverseAD1(dTdA, jacobians) with dTdA=Identity
      trace1.startReverseAD1(jacobians);
      trace2.startReverseAD1(jacobians);
    }

    /// If we are not the root, we simply pass on the adjoint matrix dFdT to all terms
    template <typename MatrixType>
    void reverseAD4(const MatrixType& dFdT, JacobianMap& jacobians) const {
      // NOTE(frank): equivalent to trace.reverseAD1(dFdT * dTdA, jacobians) with dTdA=Identity
      trace1.reverseAD1(dFdT, jacobians);
      trace2.reverseAD1(dFdT, jacobians);
    }
  };

  /// Construct an execution trace for reverse AD
  T traceExecution(const Values& values, ExecutionTrace<T>& trace,
                           ExecutionTraceStorage* ptr) const override {
    assert(reinterpret_cast<size_t>(ptr) % TraceAlignment == 0);
    Record* record = new (ptr) Record();
    trace.setFunction(record);

    ExecutionTraceStorage* ptr1 = ptr + upAligned(sizeof(Record));
    ExecutionTraceStorage* ptr2 = ptr1 + expression1_->traceSize();
    return expression1_->traceExecution(values, record->trace1, ptr1) +
           expression2_->traceExecution(values, record->trace2, ptr2);
  }
};

}  // namespace internal
}  // namespace gtsam
