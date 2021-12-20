/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CallRecord.h
 * @date November 21, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @author Hannes Sommer
 * @brief Internals for Expression.h, not for general consumption
 */

#pragma once

#include <gtsam/nonlinear/internal/JacobianMap.h>

namespace gtsam {
namespace internal {

/**
 * ConvertToDynamicIf converts to a dense matrix with dynamic rows iff
 * ConvertToDynamicRows (colums stay as they are) otherwise
 * it just passes dense Eigen matrices through.
 */
template<bool ConvertToDynamicRows>
struct ConvertToDynamicIf {
  template<typename Derived>
  static Eigen::Matrix<double, Eigen::Dynamic, Derived::ColsAtCompileTime> convert(
      const Eigen::MatrixBase<Derived>& x) {
    return x;
  }
};

template<>
struct ConvertToDynamicIf<false> {
  template<typename Derived>
  static const Eigen::Matrix<double, Derived::RowsAtCompileTime,
      Derived::ColsAtCompileTime> convert(
      const Eigen::MatrixBase<Derived> & x) {
    return x;
  }
  // Most common case:  just pass through matrices that are already of the right fixed-size type
  template <int Rows, int Cols>
  static const Eigen::Matrix<double, Rows, Cols>& convert(
      const Eigen::Matrix<double, Rows, Cols>& x) {
    return x;
  }
};

/**
 * The CallRecord is an abstract base class for any class that stores
 * the Jacobians of applying a function with respect to each of its arguments,
 * as well as an execution trace for each of its arguments.
 *
 * The complicated structure of this class is to get around the limitations of mixing inheritance
 * (needed so that a trace can keep Record pointers) and templating (needed for efficient matrix
 * multiplication). The "hack" is to implement N differently sized reverse AD methods, and select
 * the appropriate version with the dispatch method reverseAD2 below.
 */
template<int Cols>
struct CallRecord {

  // Print entire record, recursively
  inline void print(const std::string& indent) const {
    _print(indent);
  }

  // Main entry point for the reverse AD process of a functional expression.
  // Called *once* by the main AD entry point, ExecutionTrace::startReverseAD1
  // This function then calls ExecutionTrace::reverseAD for every argument
  // which will in turn call the reverseAD method below.
  // Calls virtual function _startReverseAD3, implemented in derived
  inline void startReverseAD2(JacobianMap& jacobians) const {
    _startReverseAD3(jacobians);
  }

  // Dispatch the reverseAD2 calls issued by ExecutionTrace::reverseAD1
  // Here we convert dFdT to a dynamic Matrix if the # rows>5, because _reverseAD3 is only
  // specialized for fixed-size matrices up to 5 rows.
  // The appropriate _reverseAD3 method is selected by method overloading.
  template <typename Derived>
  inline void reverseAD2(const Eigen::MatrixBase<Derived>& dFdT, JacobianMap& jacobians) const {
    _reverseAD3(ConvertToDynamicIf<(Derived::RowsAtCompileTime > 5)>::convert(dFdT), jacobians);
  }

  // This overload supports matrices with both rows and columns dynamically sized.
  // The template version above would be slower by introducing an extra conversion
  // to statically sized columns.
  inline void reverseAD2(const Matrix & dFdT, JacobianMap& jacobians) const {
    _reverseAD3(dFdT, jacobians);
  }

  virtual ~CallRecord() {
  }

private:

  virtual void _print(const std::string& indent) const = 0;
  virtual void _startReverseAD3(JacobianMap& jacobians) const = 0;

  virtual void _reverseAD3(const Matrix & dFdT,
      JacobianMap& jacobians) const = 0;

  virtual void _reverseAD3(
      const Eigen::Matrix<double, Eigen::Dynamic, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;

  virtual void _reverseAD3(const Eigen::Matrix<double, 1, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
  virtual void _reverseAD3(const Eigen::Matrix<double, 2, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
  virtual void _reverseAD3(const Eigen::Matrix<double, 3, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
  virtual void _reverseAD3(const Eigen::Matrix<double, 4, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
  virtual void _reverseAD3(const Eigen::Matrix<double, 5, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
};

/**
 * CallRecordMaxVirtualStaticRows tells which separate virtual reverseAD with specific
 * static rows (1..CallRecordMaxVirtualStaticRows) methods are part of the CallRecord
 * interface. It is used to keep the testCallRecord unit test in sync.
 */
const int CallRecordMaxVirtualStaticRows = 5;

/**
 * The CallRecordImplementor implements the CallRecord interface for a Derived class by
 * delegating to its corresponding (templated) non-virtual methods.
 */
template<typename Derived, int Cols>
struct CallRecordImplementor: public CallRecord<Cols> {
private:

  const Derived & derived() const {
    return static_cast<const Derived&>(*this);
  }

  void _print(const std::string& indent) const override {
    derived().print(indent);
  }

  // Called from base class non-virtual inline method startReverseAD2
  // Calls non-virtual function startReverseAD4, implemented in Derived (ExpressionNode::Record)
  void _startReverseAD3(JacobianMap& jacobians) const override {
    derived().startReverseAD4(jacobians);
  }

  void _reverseAD3(const Matrix & dFdT, JacobianMap& jacobians) const override {
    derived().reverseAD4(dFdT, jacobians);
  }

  void _reverseAD3(
      const Eigen::Matrix<double, Eigen::Dynamic, Cols> & dFdT,
      JacobianMap& jacobians) const override {
    derived().reverseAD4(dFdT, jacobians);
  }
  void _reverseAD3(const Eigen::Matrix<double, 1, Cols> & dFdT,
      JacobianMap& jacobians) const override {
    derived().reverseAD4(dFdT, jacobians);
  }
  void _reverseAD3(const Eigen::Matrix<double, 2, Cols> & dFdT,
      JacobianMap& jacobians) const override {
    derived().reverseAD4(dFdT, jacobians);
  }
  void _reverseAD3(const Eigen::Matrix<double, 3, Cols> & dFdT,
      JacobianMap& jacobians) const override {
    derived().reverseAD4(dFdT, jacobians);
  }
  void _reverseAD3(const Eigen::Matrix<double, 4, Cols> & dFdT,
      JacobianMap& jacobians) const override {
    derived().reverseAD4(dFdT, jacobians);
  }
  void _reverseAD3(const Eigen::Matrix<double, 5, Cols> & dFdT,
      JacobianMap& jacobians) const override {
    derived().reverseAD4(dFdT, jacobians);
  }
};

} // namespace internal
} // gtsam
