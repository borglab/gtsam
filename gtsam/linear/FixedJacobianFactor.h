/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FixedJacobianFactor.h
 * @brief Arbitrary-arity Jacobian factor with compile-time block dimensions.
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/timing.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/internal/FixedJacobianFactorOps.h>

#include <algorithm>
#include <array>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace gtsam {
namespace internal {

/** Return the element at I in a compile-time integer pack, or Default. */
template <size_t I, int Default, int... Dimensions>
struct DimensionAt : std::integral_constant<int, Default> {};

template <size_t I, int Default, int First, int... Rest>
struct DimensionAt<I, Default, First, Rest...>
    : DimensionAt<I - 1, Default, Rest...> {};

template <int Default, int First, int... Rest>
struct DimensionAt<0, Default, First, Rest...>
    : std::integral_constant<int, First> {};

/** Compile-time residual and variable dimensions for a factor of any arity. */
template <typename OutputVec, typename... ValueTypes>
struct FixedSizeDimensions {
  inline constexpr static size_t arity = sizeof...(ValueTypes);
  inline constexpr static int M = traits<OutputVec>::dimension;
  inline constexpr static std::array<int, arity + 1> dimensions{
      M, traits<ValueTypes>::dimension...};
  template <size_t I>
  inline constexpr static int N = dimensions.at(I + 1);
  inline constexpr static bool allFixed =
      M != Eigen::Dynamic &&
      ((traits<ValueTypes>::dimension != Eigen::Dynamic) && ...);
};

/** Check dynamic Jacobian blocks against their compile-time dimensions. */
template <typename Dimensions, size_t... Is>
bool dimensionsMatch(const std::vector<Matrix>& jacobians,
                     std::index_sequence<Is...>) {
  return ((jacobians[Is].rows() == Dimensions::M &&
           jacobians[Is].cols() == Dimensions::template N<Is>) &&
          ...);
}

}  // namespace internal

/**
 * A JacobianFactor whose residual and variable dimensions are known at compile
 * time. Fixed-size kernels are used for any number of variable blocks.
 */
template <int M, int... Ns>
struct FixedJacobianFactor : JacobianFactor {
  /// Number of variable blocks in the factor.
  inline constexpr static size_t arity = sizeof...(Ns);

  /// Compile-time dimension of each variable block.
  inline constexpr static std::array<int, arity> dimensions{Ns...};

 private:
  template <size_t I>
  inline constexpr static int N = internal::DimensionAt<I, 1, Ns...>::value;

  inline constexpr static std::array<DenseIndex, arity + 1> offsets = [] {
    std::array<DenseIndex, arity + 1> result{};
    for (size_t i = 0; i < arity; ++i) {
      result[i + 1] = result[i] + dimensions[i];
    }
    return result;
  }();

  static VerticalBlockMatrix augmentedMatrix(
      const std::vector<Matrix>& jacobians, const Vector& b) {
    if (jacobians.size() != arity || b.size() != M) {
      throw std::invalid_argument(
          "FixedJacobianFactor: error or Jacobian count mismatch");
    }

    const std::array<size_t, arity> blockDimensions{static_cast<size_t>(Ns)...};
    VerticalBlockMatrix result(blockDimensions, M, true);
    for (size_t i = 0; i < arity; ++i) {
      if (jacobians[i].rows() != M || jacobians[i].cols() != dimensions[i]) {
        throw std::invalid_argument(
            "FixedJacobianFactor: Jacobian dimension mismatch");
      }
      result(i) = jacobians[i];
    }
    result(arity) = b;
    return result;
  }

  template <size_t I>
  internal::FixedJacobianBlock<M, N<I>> block() const {
    return internal::FixedJacobianBlock<M, N<I>>(Ab_.matrix(), 0, offsets[I]);
  }

  template <size_t... Is>
  void hessianDiagonalAdd(VectorValues& diagonal,
                          std::index_sequence<Is...>) const {
    (internal::accumulateHessianDiagonal<M, N<Is>>(keys_[Is], block<Is>(),
                                                   diagonal),
     ...);
  }

  template <size_t... Is>
  void accumulateResidual(const VectorValues& values,
                          Eigen::Matrix<double, M, 1>& residual,
                          std::index_sequence<Is...>) const {
    (internal::accumulateResidual<M, N<Is>>(block<Is>(), values.at(keys_[Is]),
                                            residual),
     ...);
  }

  template <size_t... Is>
  std::array<DenseIndex, arity> slots(const KeyVector& infoKeys,
                                      std::index_sequence<Is...>) const {
    return {{Slot(infoKeys, keys_[Is])...}};
  }

  template <size_t I, size_t... Js>
  void updateCrossHessiansFor(const std::array<DenseIndex, arity>& factorSlots,
                              SymmetricBlockMatrix* info,
                              std::index_sequence<Js...>) const {
    (internal::updateCrossHessian<M, N<I>, N<I + Js + 1>>(
         factorSlots[I], block<I>(), factorSlots[I + Js + 1],
         block<I + Js + 1>(), info),
     ...);
  }

  template <size_t... Is>
  void updateCrossHessians(const std::array<DenseIndex, arity>& factorSlots,
                           SymmetricBlockMatrix* info,
                           std::index_sequence<Is...>) const {
    (updateCrossHessiansFor<Is>(factorSlots, info,
                                std::make_index_sequence<arity - Is - 1>{}),
     ...);
  }

  template <size_t... Is>
  void updateJacobianHessians(const std::array<DenseIndex, arity>& factorSlots,
                              DenseIndex slotB,
                              const internal::FixedJacobianBlock<M, 1>& b,
                              SymmetricBlockMatrix* info,
                              std::index_sequence<Is...>) const {
    (internal::updateJacobianHessian<M, N<Is>>(factorSlots[Is], block<Is>(),
                                               slotB, b, info),
     ...);
  }

  template <size_t I, typename OwnsColumn, size_t... Js>
  void updateOwnedCrossHessiansFor(
      const std::array<DenseIndex, arity>& factorSlots,
      const OwnsColumn& ownsColumn, SymmetricBlockMatrix* info,
      std::index_sequence<Js...>) const {
    ((ownsColumn(std::max(factorSlots[I], factorSlots[I + Js + 1]))
          ? static_cast<void>(
                internal::updateCrossHessian<M, N<I>, N<I + Js + 1>>(
                    factorSlots[I], block<I>(), factorSlots[I + Js + 1],
                    block<I + Js + 1>(), info))
          : static_cast<void>(0)),
     ...);
  }

  template <typename OwnsColumn, size_t... Is>
  void updateOwnedCrossHessians(
      const std::array<DenseIndex, arity>& factorSlots,
      const OwnsColumn& ownsColumn, SymmetricBlockMatrix* info,
      std::index_sequence<Is...>) const {
    (updateOwnedCrossHessiansFor<Is>(
         factorSlots, ownsColumn, info,
         std::make_index_sequence<arity - Is - 1>{}),
     ...);
  }

  template <typename OwnsColumn, size_t... Is>
  void updateOwnedSelfAndRhsHessians(
      const std::array<DenseIndex, arity>& factorSlots, DenseIndex slotB,
      const internal::FixedJacobianBlock<M, 1>& b, const OwnsColumn& ownsColumn,
      SymmetricBlockMatrix* info, std::index_sequence<Is...>) const {
    ((ownsColumn(factorSlots[Is])
          ? static_cast<void>(internal::updateSelfHessian<M, N<Is>>(
                factorSlots[Is], block<Is>(), info))
          : static_cast<void>(0)),
     ...);
    ((ownsColumn(std::max(factorSlots[Is], slotB))
          ? static_cast<void>(internal::updateCrossHessian<M, N<Is>, 1>(
                factorSlots[Is], block<Is>(), slotB, b, info))
          : static_cast<void>(0)),
     ...);
  }

 public:
  static_assert(arity > 0 && M > 0 && ((Ns > 0) && ...),
                "Fixed factor arity and dimensions must be positive");

  /**
   * Construct from keys, Jacobian blocks, and a right-hand side.
   *
   * The number and dimensions of the dynamic Jacobian matrices are checked
   * against the compile-time dimensions.
   */
  FixedJacobianFactor(const KeyVector& keys,
                      const std::vector<Matrix>& jacobians, const Vector& b,
                      const SharedDiagonal& model = SharedDiagonal())
      : JacobianFactor(keys, augmentedMatrix(jacobians, b), model) {}

  /** Add the Hessian diagonal using fixed-size matrix operations. */
  void hessianDiagonalAdd(VectorValues& diagonal) const override {
    const SharedDiagonal& model = get_model();
    if (model && !model->isUnit()) {
      JacobianFactor::hessianDiagonalAdd(diagonal);
      return;
    }
    hessianDiagonalAdd(diagonal, std::make_index_sequence<arity>{});
  }

  /** Compute the error change using a fixed-size residual. */
  double deltaError(const VectorValues& values, double* oldError = nullptr,
                    double* newError = nullptr) const override {
    const SharedDiagonal& model = get_model();
    if (model && !model->isUnit()) {
      return JacobianFactor::deltaError(values, oldError, newError);
    }

    const internal::FixedJacobianBlock<M, 1> b(Ab_.matrix(), 0, offsets[arity]);
    Eigen::Matrix<double, M, 1> error = -b;
    accumulateResidual(values, error, std::make_index_sequence<arity>{});

    const double oldValue = 0.5 * b.squaredNorm();
    const double newValue = 0.5 * error.squaredNorm();
    if (oldError) *oldError = oldValue;
    if (newError) *newError = newValue;
    return oldValue - newValue;
  }

  /** Update the complete augmented Hessian using fixed-size products. */
  void updateHessian(const KeyVector& infoKeys,
                     SymmetricBlockMatrix* info) const override {
    gttic(updateHessian_FixedJacobianFactor);
    const SharedDiagonal& model = get_model();
    if (model && !model->isUnit()) {
      if (model->isConstrained()) {
        throw std::invalid_argument(
            "FixedJacobianFactor::updateHessian: cannot update information "
            "with constrained noise model");
      }
      std::vector<Matrix> whitenedJacobians;
      whitenedJacobians.reserve(arity);
      for (auto variable = begin(); variable != end(); ++variable) {
        whitenedJacobians.push_back(model->Whiten(getA(variable)));
      }
      const FixedJacobianFactor whitenedFactor(keys_, whitenedJacobians,
                                               model->whiten(getb()));
      whitenedFactor.updateHessian(infoKeys, info);
      return;
    }

    const auto factorSlots = slots(infoKeys, std::make_index_sequence<arity>{});
    const DenseIndex slotB = info->nBlocks() - 1;
    const internal::FixedJacobianBlock<M, 1> b(Ab_.matrix(), 0, offsets[arity]);
    updateJacobianHessians(factorSlots, slotB, b, info,
                           std::make_index_sequence<arity>{});
    if constexpr (arity > 1) {
      updateCrossHessians(factorSlots, info,
                          std::make_index_sequence<arity - 1>{});
    }
    internal::updateSelfHessian<M, 1>(slotB, b, info);
  }

  /** Update selected augmented-Hessian block columns. */
  void updateHessian(const KeyVector& infoKeys, SymmetricBlockMatrix* info,
                     DenseIndex beginCol, DenseIndex endCol) const override {
    gttic(updateHessianRange_FixedJacobianFactor);
    const SharedDiagonal& model = get_model();
    if (model && !model->isUnit()) {
      JacobianFactor::updateHessian(infoKeys, info, beginCol, endCol);
      return;
    }

    const auto factorSlots = slots(infoKeys, std::make_index_sequence<arity>{});
    const DenseIndex slotB = info->nBlocks() - 1;
    const auto ownsColumn = [beginCol, endCol](DenseIndex column) {
      return column >= beginCol && column < endCol;
    };
    const internal::FixedJacobianBlock<M, 1> b(Ab_.matrix(), 0, offsets[arity]);
    updateOwnedSelfAndRhsHessians(factorSlots, slotB, b, ownsColumn, info,
                                  std::make_index_sequence<arity>{});
    if constexpr (arity > 1) {
      updateOwnedCrossHessians(factorSlots, ownsColumn, info,
                               std::make_index_sequence<arity - 1>{});
    }
    if (ownsColumn(slotB)) {
      internal::updateSelfHessian<M, 1>(slotB, b, info);
    }
  }
};

template <int M, int... Ns>
struct traits<FixedJacobianFactor<M, Ns...>>
    : Testable<FixedJacobianFactor<M, Ns...>> {};

namespace internal {

/** Construct the fixed-size Jacobian factor for a dimension pack. */
template <int M, int... Ns>
struct FixedJacobianFactorFactory {
  /// Whether every factor dimension is fixed at compile time.
  inline constexpr static bool available =
      M != Eigen::Dynamic && ((Ns != Eigen::Dynamic) && ...);

  /// Construct a fixed-size factor behind the GaussianFactor interface.
  static std::shared_ptr<GaussianFactor> create(
      const KeyVector& keys, const std::vector<Matrix>& jacobians,
      const Vector& b, const SharedDiagonal& model) {
    return std::make_shared<FixedJacobianFactor<M, Ns...>>(keys, jacobians, b,
                                                           model);
  }
};

}  // namespace internal
}  // namespace gtsam
