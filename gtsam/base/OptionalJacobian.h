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
 * Below this class, a dynamic version is also implemented.
 */
template<int Rows, int Cols>
class OptionalJacobian {

public:

  /// ::Jacobian size type
  typedef Eigen::Matrix<double, Rows, Cols> Jacobian;

private:

  Eigen::Map<Jacobian> map_; /// View on constructor argument, if given

  // Trick from http://eigen.tuxfamily.org/dox/group__TutorialMapClass.html
  // uses "placement new" to make map_ usurp the memory of the fixed size matrix
  void usurp(double* data) {
    new (&map_) Eigen::Map<Jacobian>(data);
  }

public:

  /// Default constructor acts like boost::none
  OptionalJacobian() :
      map_(NULL) {
  }

  /// Constructor that will usurp data of a fixed-size matrix
  OptionalJacobian(Jacobian& fixed) :
      map_(NULL) {
    usurp(fixed.data());
  }

  /// Constructor that will usurp data of a fixed-size matrix, pointer version
  OptionalJacobian(Jacobian* fixedPtr) :
      map_(NULL) {
    if (fixedPtr)
      usurp(fixedPtr->data());
  }

  /// Constructor that will resize a dynamic matrix (unless already correct)
  OptionalJacobian(Eigen::MatrixXd& dynamic) :
      map_(NULL) {
    dynamic.resize(Rows, Cols); // no malloc if correct size
    usurp(dynamic.data());
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
      usurp(optional->data());
    }
  }

#endif

  /// Return true is allocated, false if default constructor was used
  operator bool() const {
    return map_.data() != NULL;
  }

  /// De-reference, like boost optional
  Eigen::Map<Jacobian>& operator*() {
    return map_;
  }

  /// TODO: operator->()
  Eigen::Map<Jacobian>* operator->(){ return &map_; }
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
    pointer_(NULL) {
  }

  /// Constructor that will resize a dynamic matrix (unless already correct)
  OptionalJacobian(Eigen::MatrixXd& dynamic) :
      pointer_(&dynamic) {
  }

#ifndef OPTIONALJACOBIAN_NOBOOST

  /// Constructor with boost::none just makes empty
  OptionalJacobian(boost::none_t none) :
    pointer_(NULL) {
  }

  /// Constructor compatible with old-style derivatives
  OptionalJacobian(const boost::optional<Eigen::MatrixXd&> optional) :
      pointer_(NULL) {
    if (optional) pointer_ = &(*optional);
  }

#endif

  /// Return true is allocated, false if default constructor was used
  operator bool() const {
    return pointer_!=NULL;
  }

  /// De-reference, like boost optional
  Jacobian& operator*() {
    return *pointer_;
  }

  /// TODO: operator->()
  Jacobian* operator->(){ return pointer_; }
};

} // namespace gtsam

