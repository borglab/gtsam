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

#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/base/Matrix.h>

#include <boost/mpl/transform.hpp>

namespace gtsam {

class JacobianMap;
// forward declaration

//-----------------------------------------------------------------------------
/**
 * MaxVirtualStaticRows defines how many separate virtual reverseAD with specific
 * static rows (1..MaxVirtualStaticRows) methods will be part of the CallRecord interface.
 */
#define MaxVirtualStaticRows 4

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
  static const Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> convert(
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

} // namespace internal

/**
 * The CallRecord class stores the Jacobians of applying a function
 * with respect to each of its arguments. It also stores an execution trace
 * (defined below) for each of its arguments.
 *
 * It is implemented in the function-style ExpressionNode's nested Record class below.
 */
template<int Cols>
struct CallRecord {
  inline void print(const std::string& indent) const {
    _print(indent);
  }

  inline void startReverseAD(JacobianMap& jacobians) const {
    _startReverseAD(jacobians);
  }

  template<typename Derived>
  inline void reverseAD(const Eigen::MatrixBase<Derived> & dFdT,
      JacobianMap& jacobians) const {
    _reverseAD(
         internal::ConvertToVirtualFunctionSupportedMatrixType<
             (Derived::RowsAtCompileTime > MaxVirtualStaticRows)
           >::convert(dFdT),
         jacobians
       );
  }

  inline void reverseAD(const Matrix & dFdT, JacobianMap& jacobians) const {
    _reverseAD(dFdT, jacobians);
  }

  virtual ~CallRecord() {
  }

private:
  virtual void _print(const std::string& indent) const = 0;
  virtual void _startReverseAD(JacobianMap& jacobians) const = 0;

  virtual void _reverseAD(const Matrix & dFdT, JacobianMap& jacobians) const = 0;
  virtual void _reverseAD(
      const Eigen::Matrix<double, Eigen::Dynamic, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
#if MaxVirtualStaticRows >= 1
  virtual void _reverseAD(
      const Eigen::Matrix<double, 1, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
#endif
#if MaxVirtualStaticRows >= 2
  virtual void _reverseAD(
      const Eigen::Matrix<double, 2, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
#endif
#if MaxVirtualStaticRows >= 3
  virtual void _reverseAD(
      const Eigen::Matrix<double, 3, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
#endif
#if MaxVirtualStaticRows >= 4
  virtual void _reverseAD(
      const Eigen::Matrix<double, 4, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
#endif
#if MaxVirtualStaticRows >= 5
  virtual void _reverseAD(
      const Eigen::Matrix<double, 5, Cols> & dFdT,
      JacobianMap& jacobians) const = 0;
#endif
};

namespace internal {
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
  virtual void _startReverseAD(JacobianMap& jacobians) const {
    derived().startReverseAD(jacobians);
  }

  virtual void _reverseAD(const Matrix & dFdT, JacobianMap& jacobians) const {
    derived().reverseAD(dFdT, jacobians);
  }
  virtual void _reverseAD(
      const Eigen::Matrix<double, Eigen::Dynamic, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD(dFdT, jacobians);
  }
#if MaxVirtualStaticRows >= 1
  virtual void _reverseAD(
      const Eigen::Matrix<double, 1, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD(dFdT, jacobians);
  }
#endif
#if MaxVirtualStaticRows >= 2
  virtual void _reverseAD(
      const Eigen::Matrix<double, 2, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD(dFdT, jacobians);
  }
#endif
#if MaxVirtualStaticRows >= 3
  virtual void _reverseAD(
      const Eigen::Matrix<double, 3, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD(dFdT, jacobians);
  }
#endif
#if MaxVirtualStaticRows >= 4
  virtual void _reverseAD(
      const Eigen::Matrix<double, 4, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD(dFdT, jacobians);
  }
#endif
#if MaxVirtualStaticRows >= 5
  virtual void _reverseAD(
      const Eigen::Matrix<double, 5, Cols> & dFdT,
      JacobianMap& jacobians) const {
    derived().reverseAD(dFdT, jacobians);
  }
#endif
};

} // namespace internal

} // gtsam
