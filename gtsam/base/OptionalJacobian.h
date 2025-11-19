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
#include <cstddef>
#include <functional>
#include <gtsam/config.h>      // Configuration from CMake
#include <Eigen/Dense>
#include <optional>
#include <stdexcept>
#include <string>

namespace gtsam {

/**
 * OptionalJacobian is an Eigen::Ref like class that can take be constructed using
 * either a fixed size or dynamic Eigen matrix. In the latter case, the dynamic
 * matrix will be resized.
 * Below this class, a dynamic version is also implemented.
 */
template<int Rows, int Cols>
class OptionalJacobian {

public:

  /// Jacobian size type
  /// TODO(frank): how to enforce RowMajor? Or better, make it work with any storage order?
  typedef Eigen::Matrix<double, Rows, Cols> Jacobian;
  typedef Eigen::Stride<Eigen::Dynamic, 1> Stride;
  typedef Eigen::Map<Jacobian, 0, Stride> Map;

private:

  Map map_; /// View on constructor argument, if given

  // Trick from http://eigen.tuxfamily.org/dox/group__TutorialMapClass.html
  // uses "placement new" to make map_ usurp the memory of the fixed size matrix
  void usurp(double* data, Eigen::Index outerStride) {
    new (&map_) Map(data, Stride(outerStride, 1));
  }

  // Private and very dangerous constructor straight from memory
  OptionalJacobian(double* data, Eigen::Index outerStride)
      : map_(nullptr, Stride(1, 1)) {
    if (data) usurp(data, outerStride);
  }

  template<int M, int N>
  friend class OptionalJacobian;

public:

  /// Default constructor
  OptionalJacobian() : map_(nullptr, Stride(1, 1)) {
  }

  /// Default constructor with nullptr_t
  /// To guide the compiler when nullptr
  /// is passed to args of the type OptionalJacobian
  OptionalJacobian(std::nullptr_t /*unused*/) :
      map_(nullptr, Stride(1, 1)) {
  }

  /// Constructor that will usurp data of a fixed-size matrix
  OptionalJacobian(Jacobian& fixed) :
      map_(nullptr, Stride(1, 1)) {
    usurp(fixed.data(), fixed.rows());
  }

  /// Constructor that will usurp data of a fixed-size matrix, pointer version
  OptionalJacobian(Jacobian* fixedPtr) :
      map_(nullptr, Stride(1, 1)) {
    if (fixedPtr)
      usurp(fixedPtr->data(), fixedPtr->rows());
  }

  /// Constructor that will resize a dynamic matrix (unless already correct)
  OptionalJacobian(Eigen::MatrixXd& dynamic) :
      map_(nullptr, Stride(1, 1)) {
    dynamic.resize(Rows, Cols); // no malloc if correct size
    usurp(dynamic.data(), dynamic.rows());
  }

  /// Constructor that will resize a dynamic matrix (unless already correct)
  OptionalJacobian(Eigen::MatrixXd* dynamic) :
      map_(nullptr, Stride(1, 1)) {
    if (dynamic) {
      dynamic->resize(Rows, Cols);  // no malloc if correct size
      usurp(dynamic->data(), dynamic->rows());
    }
  }

  /**
   * @brief Constructor from an Eigen::Ref *value*. Will not usurp if dimension is wrong
   * @note This is important so we don't overwrite someone else's memory!
   */
  template<class MATRIX>
  OptionalJacobian(Eigen::Ref<MATRIX> dynamic_ref) :
      map_(nullptr, Stride(1, 1)) {
    if (dynamic_ref.rows() == Rows && dynamic_ref.cols() == Cols &&
        !dynamic_ref.IsRowMajor && dynamic_ref.innerStride() == 1) {
      usurp(dynamic_ref.data(), dynamic_ref.outerStride());
    } else {
      throw std::invalid_argument(
          std::string("OptionalJacobian called with wrong dimensions or "
                      "storage order.\n"
                      "Expected: ") +
          "(" + std::to_string(Rows) + ", " + std::to_string(Cols) + ")");
    }
  } 

  /// Constructor with std::nullopt just makes empty
  OptionalJacobian(std::nullopt_t /*none*/) :
      map_(nullptr, Stride(1, 1)) {
  }

  /// Constructor compatible with old-style derivatives
  OptionalJacobian(const std::optional<std::reference_wrapper<Eigen::MatrixXd>> optional) :
      map_(nullptr, Stride(1, 1)) {
    if (optional) {
      optional->get().resize(Rows, Cols);
      usurp(optional->get().data(), optional->get().rows());
    }
  }
  /// Constructor that will usurp data of a block expression
  /// TODO(frank): unfortunately using a Map makes usurping non-contiguous memory impossible
  //  template <typename Derived, bool InnerPanel>
  //  OptionalJacobian(Eigen::Block<Derived,Rows,Cols,InnerPanel> block) : map_(nullptr) { ?? }

  /// Return true if allocated, false if default constructor was used
  operator bool() const {
    return map_.data() != nullptr;
  }

  /// De-reference, like boost optional
  Map& operator*() {
    return map_;
  }

  /// operator->()
  Map* operator->() {
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
      return OptionalJacobian<Rows, N>(&map_(0, startCol), map_.outerStride());
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

  /// Default constructor 
  OptionalJacobian() :
    pointer_(nullptr) {
  }

  /// Construct from pointer to dynamic matrix
  OptionalJacobian(Jacobian* pointer) : pointer_(pointer) {}

  /// Construct from refrence to dynamic matrix
  OptionalJacobian(Jacobian& dynamic) : pointer_(&dynamic) {}

  /// Constructor with std::nullopt just makes empty
  OptionalJacobian(std::nullopt_t /*none*/) :
    pointer_(nullptr) {
  }

  /// Constructor for optional matrix reference
  OptionalJacobian(const std::optional<std::reference_wrapper<Eigen::MatrixXd>> optional) :
      pointer_(nullptr) {
    if (optional) pointer_ = &((*optional).get());
  }

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
