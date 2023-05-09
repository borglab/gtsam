/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    OptionalJacobian.h
 * @brief   Special class for optional Jacobian arguments
 * @author  Frank Dellaert
 * @author  Natesh Srinivasan
 * @date    Nov 28, 2014
 */

#pragma once
#include <gtsam/config.h>      // Configuration from CMake
#include <Eigen/Dense>

#ifndef OPTIONALJACOBIAN_NOBOOST
#include <boost/optional.hpp>
#endif

namespace gtsam {

/**
 * OptionalJacobian is an Eigen::Ref like class that can take be constructed using
 * either a fixed size or dynamic Eigen matrix. In the latter case, the dynamic
 * matrix will be resized. Finally, there is a constructor that takes
 * boost::none, the default constructor acts like boost::none, and
 * boost::optional<Eigen::MatrixXd&> is also supported for backwards compatibility.
 * Below this class, a dynamic version is also implemented.
 */
template<int Rows, int Cols>
class OptionalJacobian {

public:

  /// Jacobian size type
  /// TODO(frank): how to enforce RowMajor? Or better, make it work with any storage order?
  typedef Eigen::Matrix<double, Rows, Cols> Jacobian;

private:

  Eigen::Map<Jacobian> map_; /// View on constructor argument, if given

  // Trick from http://eigen.tuxfamily.org/dox/group__TutorialMapClass.html
  // uses "placement new" to make map_ usurp the memory of the fixed size matrix
  void usurp(double* data) {
    new (&map_) Eigen::Map<Jacobian>(data);
  }

  // Private and very dangerous constructor straight from memory
  OptionalJacobian(double* data) : map_(nullptr) {
    if (data) usurp(data);
  }

  template<int M, int N>
  friend class OptionalJacobian;

public:

  /// Default constructor acts like boost::none
  OptionalJacobian() :
      map_(nullptr) {
  }

  /// Constructor that will usurp data of a fixed-size matrix
  OptionalJacobian(Jacobian& fixed) :
      map_(nullptr) {
    usurp(fixed.data());
  }

  /// Constructor that will usurp data of a fixed-size matrix, pointer version
  OptionalJacobian(Jacobian* fixedPtr) :
      map_(nullptr) {
    if (fixedPtr)
      usurp(fixedPtr->data());
  }

  /// Constructor that will resize a dynamic matrix (unless already correct)
  OptionalJacobian(Eigen::MatrixXd& dynamic) :
      map_(nullptr) {
    dynamic.resize(Rows, Cols); // no malloc if correct size
    usurp(dynamic.data());
  }

  /// Constructor that will resize a dynamic matrix (unless already correct)
  OptionalJacobian(Eigen::MatrixXd* dynamic) :
      map_(nullptr) {
    dynamic->resize(Rows, Cols); // no malloc if correct size
    usurp(dynamic->data());
  }

#ifndef OPTIONALJACOBIAN_NOBOOST

  /// Constructor with boost::none just makes empty
  OptionalJacobian(boost::none_t /*none*/) :
      map_(nullptr) {
  }

  /// Constructor compatible with old-style derivatives
  OptionalJacobian(const boost::optional<Eigen::MatrixXd&> optional) :
      map_(nullptr) {
    if (optional) {
      optional->resize(Rows, Cols);
      usurp(optional->data());
    }
  }

#endif

  /// Constructor that will usurp data of a block expression
  /// TODO(frank): unfortunately using a Map makes usurping non-contiguous memory impossible
  //  template <typename Derived, bool InnerPanel>
  //  OptionalJacobian(Eigen::Block<Derived,Rows,Cols,InnerPanel> block) : map_(nullptr) { ?? }

  /// Return true if allocated, false if default constructor was used
  operator bool() const {
    return map_.data() != nullptr;
  }

  /// De-reference, like boost optional
  Eigen::Map<Jacobian>& operator*() {
    return map_;
  }

  /// operator->()
  Eigen::Map<Jacobian>* operator->() {
    return &map_;
  }

  /// Access M*N sub-block if we are allocated, otherwise none
  /// TODO(frank): this could work as is below if only constructor above worked
  //  template <int M, int N>
  //  OptionalJacobian<M, N> block(int startRow, int startCol) {
  //    if (*this)
  //      OptionalJacobian<M, N>(map_.block<M, N>(startRow, startCol));
  //    else
  //      return OptionalJacobian<M, N>();
  //  }

  /// Access Rows*N sub-block if we are allocated, otherwise return an empty OptionalJacobian
  /// The use case is functions with arguments that are dissected, e.g. Pose3 into Rot3, Point3
  /// TODO(frank): ideally, we'd like full block functionality, but see note above.
  template <int N>
  OptionalJacobian<Rows, N> cols(int startCol) {
    if (*this)
      return OptionalJacobian<Rows, N>(&map_(0,startCol));
    else
      return OptionalJacobian<Rows, N>();
  }

  /// Access M*Cols sub-block if we are allocated, otherwise return empty OptionalJacobian
  /// The use case is functions that create their return value piecemeal by calling other functions
  /// TODO(frank): Unfortunately we assume column-major storage order and hence this can't work
  /// template <int M> OptionalJacobian<M, Cols> rows(int startRow) { ?? }
};

// The pure dynamic specialization of this is needed to support
// variable-sized types. Note that this is designed to work like the
// boost optional scheme from GTSAM 3.
template<>
class OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> {

public:

  /// Jacobian size type
  typedef Eigen::MatrixXd Jacobian;

private:

  Jacobian* pointer_; /// View on constructor argument, if given

public:

  /// Default constructor acts like boost::none
  OptionalJacobian() :
    pointer_(nullptr) {
  }

  /// Construct from pointer to dynamic matrix
  OptionalJacobian(Jacobian* pointer) : pointer_(pointer) {}

  /// Construct from refrence to dynamic matrix
  OptionalJacobian(Jacobian& dynamic) : pointer_(&dynamic) {}

#ifndef OPTIONALJACOBIAN_NOBOOST

  /// Constructor with boost::none just makes empty
  OptionalJacobian(boost::none_t /*none*/) :
    pointer_(nullptr) {
  }

  /// Constructor compatible with old-style derivatives
  OptionalJacobian(const boost::optional<Eigen::MatrixXd&> optional) :
      pointer_(nullptr) {
    if (optional) pointer_ = &(*optional);
  }

#endif

  /// Return true if allocated, false if default constructor was used
  operator bool() const {
    return pointer_!=nullptr;
  }

  /// De-reference, like boost optional
  Jacobian& operator*() {
    return *pointer_;
  }

  /// TODO: operator->()
  Jacobian* operator->(){ return pointer_; }
};

// forward declare
template <typename T> struct traits;

/**
 * @brief: meta-function to generate Jacobian
 * @param T return type
 * @param A argument type
 */
template <class T, class A>
struct MakeJacobian {
  typedef Eigen::Matrix<double, traits<T>::dimension, traits<A>::dimension> type;
};

/**
 * @brief: meta-function to generate JacobianTA optional reference
 * Used mainly by Expressions
 * @param T return type
 * @param A argument type
 */
template<class T, class A>
struct MakeOptionalJacobian {
  typedef OptionalJacobian<traits<T>::dimension,
      traits<A>::dimension> type;
};

} // namespace gtsam

