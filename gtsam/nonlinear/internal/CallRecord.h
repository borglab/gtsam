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
struct ConvertToVirtualFunctionSupportedMatrixType {
  template<typename Derived>
  static Eigen::Matrix<double, Eigen::Dynamic, Derived::ColsAtCompileTime> convert(
      const Eigen::MatrixBase<Derived> & x) {
    return x;
  }
};

template<>
struct ConvertToVirtualFunctionSupportedMatrixType<false> {
  template<typename Derived>
  static const Eigen::Matrix<double, Derived::RowsAtCompileTime,
      Derived::ColsAtCompileTime> convert(
      const Eigen::MatrixBase<Derived> & x) {
    return x;
  }
  // special treatment of matrices that don't need conversion
  template<int Rows, int Cols>
  static const Eigen::Matrix<double, Rows, Cols> & convert(
      const Eigen::Matrix<double, Rows, Cols> & x) {
    return x;
  }
};

/**
 * The CallRecord is an abstract base class for the any class that stores
 * the Jacobians of applying a function with respect to each of its arguments,
 * as well as an execution trace for each of its arguments.
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
  // This non-virtual function _startReverseAD3, implemented in derived
  inline void startReverseAD2(JacobianMap& jacobians) const {
    _startReverseAD3(jacobians);
  }

  // Dispatch the reverseAD2 calls issued by ExecutionTrace::reverseAD1
  // Here we convert to dynamic if the
  template<typename Derived>
  inline void reverseAD2(const Eigen::MatrixBase<Derived> & dFdT,
      JacobianMap& jacobians) const {
    _reverseAD3(
        ConvertToVirtualFunctionSupportedMatrixType<
            (Derived::RowsAtCompileTime > 5)>::convert(dFdT), jacobians);
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

  virtual void _print(const std::string& indent) const {
    derived().print(indent);
  }

  virtual void _startReverseAD3(JacobianMap& jacobians) const {
    derived().startReverseAD4(jacobians);
  }

  virtual void _reverseAD3(const Matrix & dFdT, JacobianMap& jacobians) const {
    derived().reverseAD4(dFdT, jacobians);
  }

  virtual void _reverseAD3(
      const Eigen::Matrix<double, Eigen::Dynamic, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD4(dFdT, jacobians);
  }
  virtual void _reverseAD3(const Eigen::Matrix<double, 1, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD4(dFdT, jacobians);
  }
  virtual void _reverseAD3(const Eigen::Matrix<double, 2, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD4(dFdT, jacobians);
  }
  virtual void _reverseAD3(const Eigen::Matrix<double, 3, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD4(dFdT, jacobians);
  }
  virtual void _reverseAD3(const Eigen::Matrix<double, 4, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD4(dFdT, jacobians);
  }
  virtual void _reverseAD3(const Eigen::Matrix<double, 5, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD4(dFdT, jacobians);
  }
};

} // namespace internal
} // gtsam
