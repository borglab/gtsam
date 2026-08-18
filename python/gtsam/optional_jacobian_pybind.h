#pragma once

#include <pybind11/eigen.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>

#include <memory>
#include <vector>

/* gtsam::OptionalMatrixVecType is std::vector<gtsam::Matrix>*, an input/output
 * argument used by NoiseModelFactor::unwhitenedError and by CustomFactor's
 * error callback. It must be opaque so that mutations on the Python side are
 * visible to C++, and so that the `= nullptr` default can round-trip through
 * None during function registration.
 *
 * This lives here rather than in a per-module preamble because every .i file is
 * generated into its own translation unit including only its own preamble,
 * whereas this header is included by both module templates. pybind11 requires
 * PYBIND11_MAKE_OPAQUE to be visible in every translation unit of a module that
 * touches the type; declaring it per-preamble made it opaque in custom.cpp and
 * transparent everywhere else.
 */
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Matrix>); // JacobianVector

namespace pybind11 {
namespace detail {

template <typename MatrixType>
class MutableEigenArgument {
 public:
  bool load(handle source) {
    if (source.is_none()) return true;
    if (!isinstance<array>(source)) return false;

    array_ = reinterpret_borrow<array>(source);
    if (!array_.writeable() || !array_.dtype().is(dtype::of<double>()) ||
        array_.ndim() != 2) {
      return false;
    }

    const ssize_t rows = array_.shape(0);
    const ssize_t cols = array_.shape(1);
    if constexpr (MatrixType::RowsAtCompileTime != Eigen::Dynamic) {
      if (rows != MatrixType::RowsAtCompileTime) return false;
    }
    if constexpr (MatrixType::ColsAtCompileTime != Eigen::Dynamic) {
      if (cols != MatrixType::ColsAtCompileTime) return false;
    }

    matrix_.resize(rows, cols);
    copyFromPython();
    has_value_ = true;
    return true;
  }

  ~MutableEigenArgument() {
    if (has_value_ && matrix_.rows() == array_.shape(0) &&
        matrix_.cols() == array_.shape(1)) {
      copyToPython();
    }
  }

  bool hasValue() const { return has_value_; }
  MatrixType& matrix() { return matrix_; }

 private:
  void copyFromPython() {
    const buffer_info info = array_.request();
    const auto* data = static_cast<const char*>(info.ptr);
    for (ssize_t row = 0; row < info.shape[0]; ++row) {
      for (ssize_t col = 0; col < info.shape[1]; ++col) {
        matrix_(row, col) = *reinterpret_cast<const double*>(
            data + row * info.strides[0] + col * info.strides[1]);
      }
    }
  }

  void copyToPython() {
    const buffer_info info = array_.request();
    auto* data = static_cast<char*>(info.ptr);
    for (ssize_t row = 0; row < info.shape[0]; ++row) {
      for (ssize_t col = 0; col < info.shape[1]; ++col) {
        *reinterpret_cast<double*>(data + row * info.strides[0] +
                                   col * info.strides[1]) = matrix_(row, col);
      }
    }
  }

  array array_;
  MatrixType matrix_;
  bool has_value_ = false;
};

template <typename EigenType>
struct MutableEigenCaster {
  using Type = EigenType;
  using Scalar = typename Type::Scalar;
  using props = EigenProps<Type>;

  bool load(handle source, bool convert) {
    if (source.is_none()) {
      is_none_ = true;
      return true;
    }
    if (!convert && !isinstance<array_t<Scalar>>(source)) return false;

    auto buffer = array::ensure(source);
    if (!buffer || buffer.ndim() < 1 || buffer.ndim() > 2) return false;

    auto fits = props::conformable(buffer);
    if (!fits) return false;

    value_ = Type(fits.rows, fits.cols);
    auto reference = reinterpret_steal<array>(eigen_ref_array<props>(value_));
    if (buffer.ndim() == 1) {
      reference = reference.squeeze();
    } else if (reference.ndim() == 1) {
      buffer = buffer.squeeze();
    }

    if (detail::npy_api::get().PyArray_CopyInto_(reference.ptr(),
                                                  buffer.ptr()) < 0) {
      PyErr_Clear();
      return false;
    }

    if (isinstance<array>(source)) {
      auto source_array = reinterpret_borrow<array>(source);
      if (source_array.writeable() &&
          source_array.dtype().is(dtype::of<Scalar>()) &&
          source_array.ndim() == 2 && source_array.shape(0) == value_.rows() &&
          source_array.shape(1) == value_.cols()) {
        source_array_ = std::move(source_array);
        write_back_ = true;
      }
    }
    return true;
  }

  ~MutableEigenCaster() {
    if (!write_back_ || value_.rows() != source_array_.shape(0) ||
        value_.cols() != source_array_.shape(1)) {
      return;
    }
    const buffer_info info = source_array_.request();
    auto* data = static_cast<char*>(info.ptr);
    for (ssize_t row = 0; row < info.shape[0]; ++row) {
      for (ssize_t col = 0; col < info.shape[1]; ++col) {
        *reinterpret_cast<Scalar*>(data + row * info.strides[0] +
                                   col * info.strides[1]) = value_(row, col);
      }
    }
  }

 private:
  template <typename CType>
  static handle cast_impl(CType* source, return_value_policy policy,
                          handle parent) {
    if (!source) return none().release();
    switch (policy) {
      case return_value_policy::take_ownership:
      case return_value_policy::automatic:
        return eigen_encapsulate<props>(source);
      case return_value_policy::move:
        return eigen_encapsulate<props>(new CType(std::move(*source)));
      case return_value_policy::copy:
        return eigen_array_cast<props>(*source);
      case return_value_policy::reference:
      case return_value_policy::automatic_reference:
        return eigen_ref_array<props>(*source);
      case return_value_policy::reference_internal:
        return eigen_ref_array<props>(*source, parent);
      default:
        throw cast_error("unhandled return_value_policy");
    }
  }

  void rejectNoneForValue() const {
    if (is_none_) {
      throw cast_error("None is only valid for an optional Matrix pointer");
    }
  }

 public:
  static handle cast(Type&& source, return_value_policy, handle parent) {
    return cast_impl(&source, return_value_policy::move, parent);
  }
  static handle cast(const Type&& source, return_value_policy, handle parent) {
    return cast_impl(&source, return_value_policy::move, parent);
  }
  static handle cast(Type& source, return_value_policy policy, handle parent) {
    if (policy == return_value_policy::automatic ||
        policy == return_value_policy::automatic_reference) {
      policy = return_value_policy::copy;
    }
    return cast_impl(&source, policy, parent);
  }
  static handle cast(const Type& source, return_value_policy policy,
                     handle parent) {
    if (policy == return_value_policy::automatic ||
        policy == return_value_policy::automatic_reference) {
      policy = return_value_policy::copy;
    }
    return cast_impl(&source, policy, parent);
  }
  static handle cast(Type* source, return_value_policy policy, handle parent) {
    return cast_impl(source, policy, parent);
  }
  static handle cast(const Type* source, return_value_policy policy,
                     handle parent) {
    return cast_impl(source, policy, parent);
  }

  static constexpr auto name = props::descriptor;

  operator Type*() { return is_none_ ? nullptr : &value_; }
  operator Type&() {
    this->rejectNoneForValue();
    return value_;
  }
  operator Type&&() && {
    this->rejectNoneForValue();
    return std::move(value_);
  }
  template <typename T>
  using cast_op_type = movable_cast_op_type<T>;

 private:
  Type value_;
  array source_array_;
  bool is_none_ = false;
  bool write_back_ = false;
};

template <>
struct type_caster<gtsam::Matrix> : MutableEigenCaster<gtsam::Matrix> {};

template <>
struct type_caster<gtsam::Vector> : MutableEigenCaster<gtsam::Vector> {};

template <int Rows, int Cols>
struct type_caster<gtsam::OptionalJacobian<Rows, Cols>> {
  using Type = gtsam::OptionalJacobian<Rows, Cols>;
  using Matrix = typename Type::Jacobian;
  static constexpr auto name = const_name("numpy.ndarray | None");

  bool load(handle source, bool) {
    if (!argument_.load(source)) return false;
    if (argument_.hasValue()) {
      value_ = std::make_unique<Type>(argument_.matrix());
    } else {
      value_ = std::make_unique<Type>();
    }
    return true;
  }

  operator Type*() { return value_.get(); }
  operator Type&() { return *value_; }

  // OptionalJacobian is an input/output argument. Casting is only needed to
  // materialize its empty default as Python None during function registration.
  static handle cast(const Type&, return_value_policy, handle) {
    return none().release();
  }
  template <typename T>
  using cast_op_type = pybind11::detail::cast_op_type<T>;

 private:
  MutableEigenArgument<Matrix> argument_;
  std::unique_ptr<Type> value_;
};

}  // namespace detail
}  // namespace pybind11
