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
#include <gtsam/base/Lie.h>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/bind.hpp>
#include <boost/type_traits/aligned_storage.hpp>

#include <map>

namespace gtsam {

//-----------------------------------------------------------------------------
// ExecutionTrace.h
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
// Expression-inl.h
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
Expression<T>::Expression(typename UnaryFunction<A>::type function,
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
Expression<T>::Expression(typename BinaryFunction<A1, A2>::type function,
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
    typename TernaryFunction<A1, A2, A3>::type function,
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
