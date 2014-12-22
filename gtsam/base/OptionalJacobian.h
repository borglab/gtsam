/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    OptionalJacobian.h
 * @brief   Special class for optional Matrix arguments
 * @author  Frank Dellaert
 * @author  Natesh Srinivasan
 * @date    Nov 28, 2014
 */

#pragma once

#include <gtsam/3rdparty/Eigen/Eigen/Dense>

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
 */
template<int Rows, int Cols>
class OptionalJacobian {

public:

  /// ::Fixed size type
  typedef Eigen::Matrix<double, Rows, Cols> Matrix;

private:

  Eigen::Map<Matrix> map_; /// View on constructor argument, if given

  // Trick from http://eigen.tuxfamily.org/dox/group__TutorialMapClass.html
  // uses "placement new" to make map_ usurp the memory of the fixed size matrix
  void usurp(double* data, int rows, int cols) {
    new (&map_) Eigen::Map<Matrix>(data, rows, cols);
  }

public:

  /// Default constructor acts like boost::none
  OptionalJacobian() :
      map_(NULL) {
  }

  /// Constructor that will usurp data of a fixed-size matrix
  OptionalJacobian(Matrix& fixed) :
      map_(NULL) {
    usurp(fixed.data(), fixed.rows(), fixed.cols());
  }

  /// Constructor that will usurp data of a fixed-size matrix, pointer version
  OptionalJacobian(Matrix* fixedPtr) :
      map_(NULL) {
    if (fixedPtr)
      usurp(fixedPtr->data(), fixedPtr->rows(), fixedPtr->cols());
  }

  // Disabled to deal with dynamic types.
  /// Constructor that will resize a dynamic matrix (unless already correct)
  OptionalJacobian(Eigen::MatrixXd& dynamic) :
      map_(NULL) {
    dynamic.resize(Rows, Cols); // no malloc if correct size
    usurp(dynamic.data(), dynamic.rows(), dynamic.cols());
  }

#ifndef OPTIONALJACOBIAN_NOBOOST

  /// Constructor with boost::none just makes empty
  OptionalJacobian(boost::none_t none) :
      map_(NULL) {
  }

  /// Constructor compatible with old-style derivatives
  OptionalJacobian(const boost::optional<Eigen::MatrixXd&> optional) :
      map_(NULL) {
    if (optional) {
      optional->resize(Rows, Cols);
      usurp(optional->data(), optional->rows(), optional->cols());
    }
  }

#endif

  /// Return true is allocated, false if default constructor was used
  operator bool() const {
    return map_.data();
  }

  /// De-reference, like boost optional
  Eigen::Map<Matrix>& operator*() {
    return map_;
  }

  /// TODO: operator->()
  Eigen::Map<Matrix>* operator->(){ return &map_; }
};

// The pure dynamic specialization of this is needed to support
// variable-sized types.
template<>
class OptionalJacobian<-1, -1> {

public:

  /// Fixed size type
  typedef Eigen::Matrix<double, -1, -1> Fixed;

private:

  Eigen::Map<Fixed> map_; /// View on constructor argument, if given

  // Trick from http://eigen.tuxfamily.org/dox/group__TutorialMapClass.html
  // uses "placement new" to make map_ usurp the memory of the fixed size matrix
  void usurp(double* data, int rows, int cols) {
    new (&map_) Eigen::Map<Fixed>(data, rows, cols);
  }

public:

  /// Default constructor acts like boost::none
  OptionalJacobian() :
      map_(NULL, 0, 0) {
  }

  /// Constructor that will usurp data of a fixed-size matrix
  OptionalJacobian(Fixed& fixed) :
      map_(NULL, 0, 0) {
    usurp(fixed.data(), fixed.rows(), fixed.cols());
  }

  /// Constructor that will usurp data of a fixed-size matrix, pointer version
  OptionalJacobian(Fixed* fixedPtr) :
      map_(NULL, 0, 0) {
    if (fixedPtr)
      usurp(fixedPtr->data(), fixedPtr->rows(), fixedPtr->cols());
  }

#ifndef OPTIONALJACOBIAN_NOBOOST

  /// Constructor with boost::none just makes empty
  OptionalJacobian(boost::none_t none) :
      map_(NULL, 0, 0) {
  }

  /// Constructor compatible with old-style derivatives
  OptionalJacobian(const boost::optional<Eigen::MatrixXd&> optional) :
      map_(NULL, 0, 0) {
    if (optional) {
      usurp(optional->data(), optional->rows(), optional->cols());
    }
  }

#endif

  /// Return true is allocated, false if default constructor was used
  operator bool() const {
    return map_.data();
  }

  /// De-reference, like boost optional
  Eigen::Map<Fixed>& operator*() {
    return map_;
  }

  /// TODO: operator->()
  Eigen::Map<Fixed>* operator->(){ return &map_; }
};


//// these make sure that no dynamic sized matrices are compiled.
//template<int Cols>
//class OptionalJacobian<-1, Cols> {};
//template<int Rows>
//class OptionalJacobian<Rows, -1> {};

} // namespace gtsam

