/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BatchJacobianFactor.h
 * @brief Row-sparse fixed-dimension batch Jacobian factor.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>

#include <Eigen/StdVector>
#include <array>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Common interface for compact batch Jacobian factors.
 *
 * This factor type deliberately does not derive from JacobianFactor. Generic
 * GaussianFactor methods fall back to a dense JacobianFactor conversion, while
 * MultifrontalSolver can use scatterInto() to copy only nonzero row blocks.
 */
class GTSAM_EXPORT BatchJacobianFactorBase : public GaussianFactor {
 public:
  using GaussianFactor::GaussianFactor;

  /// Number of scalar rows represented by this factor.
  virtual size_t rows() const = 0;

  /// Optional model on the compact factor. The intended fast path is whitened.
  virtual const SharedDiagonal& get_model() const = 0;

  /// Convert to a dense-block JacobianFactor for compatibility paths.
  virtual JacobianFactor toJacobianFactor() const = 0;

  /**
   * Scatter nonzero row blocks into an existing vertical block matrix.
   * @param target Destination clique matrix.
   * @param rowOffset First row to write.
   * @param targetBlockIndices Maps this factor's key slots to target slots.
   *        Negative entries are skipped, e.g. fixed constrained keys.
   * @return Number of rows written.
   */
  virtual size_t scatterInto(
      VerticalBlockMatrix& target, size_t rowOffset,
      const std::vector<DenseIndex>& targetBlockIndices) const = 0;

  void print(
      const std::string& s = "",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    toJacobianFactor().print(s, formatter);
  }

  bool equals(const GaussianFactor& factor, double tol = 1e-9) const override {
    if (const auto* batch =
            dynamic_cast<const BatchJacobianFactorBase*>(&factor)) {
      return toJacobianFactor().equals(batch->toJacobianFactor(), tol);
    }
    return toJacobianFactor().equals(factor, tol);
  }

  double error(const VectorValues& values) const override {
    return toJacobianFactor().error(values);
  }

  double deltaError(const VectorValues& values, double* oldError = nullptr,
                    double* newError = nullptr) const override {
    return toJacobianFactor().deltaError(values, oldError, newError);
  }

  Matrix augmentedJacobian() const override {
    return toJacobianFactor().augmentedJacobian();
  }

  std::pair<Matrix, Vector> jacobian() const override {
    return toJacobianFactor().jacobian();
  }

  Matrix augmentedInformation() const override {
    return toJacobianFactor().augmentedInformation();
  }

  Matrix information() const override {
    return toJacobianFactor().information();
  }

  void hessianDiagonalAdd(VectorValues& diagonal) const override {
    toJacobianFactor().hessianDiagonalAdd(diagonal);
  }

  void hessianDiagonal(double* diagonal) const override {
    toJacobianFactor().hessianDiagonal(diagonal);
  }

  std::map<Key, Matrix> hessianBlockDiagonal() const override {
    return toJacobianFactor().hessianBlockDiagonal();
  }

  GaussianFactor::shared_ptr negate() const override {
    return toJacobianFactor().negate();
  }

  void updateHessian(const KeyVector& keys,
                     SymmetricBlockMatrix* info) const override {
    toJacobianFactor().updateHessian(keys, info);
  }

  void updateHessian(const KeyVector& keys, SymmetricBlockMatrix* info,
                     DenseIndex beginCol, DenseIndex endCol) const override {
    toJacobianFactor().updateHessian(keys, info, beginCol, endCol);
  }

  void multiplyHessianAdd(double alpha, const VectorValues& x,
                          VectorValues& y) const override {
    toJacobianFactor().multiplyHessianAdd(alpha, x, y);
  }

  VectorValues gradientAtZero() const override {
    return toJacobianFactor().gradientAtZero();
  }

  void gradientAtZero(double* d) const override {
    toJacobianFactor().gradientAtZero(d);
  }

  Vector gradient(Key key, const VectorValues& x) const override {
    return toJacobianFactor().gradient(key, x);
  }
};

/**
 * Fixed-dimension row-sparse batch Jacobian factor.
 *
 * Rows are grouped by the original nonlinear factors. Each row group stores one
 * fixed-size Jacobian block per variable slot and one fixed-size RHS vector.
 */
template <int ErrorDim, int... BlockDims>
class BatchJacobianFactor : public BatchJacobianFactorBase {
 public:
  static_assert(ErrorDim != Eigen::Dynamic,
                "BatchJacobianFactor requires a fixed error dimension.");
  static_assert(((BlockDims != Eigen::Dynamic) && ...),
                "BatchJacobianFactor requires fixed block dimensions.");

  using Base = BatchJacobianFactorBase;
  using This = BatchJacobianFactor<ErrorDim, BlockDims...>;
  using shared_ptr = std::shared_ptr<This>;
  static constexpr size_t NumSlots = sizeof...(BlockDims);
  using SlotIndices = std::array<DenseIndex, NumSlots>;
  using RhsVector = Eigen::Matrix<double, ErrorDim, 1>;
  template <int BlockDim>
  using BlockMatrix = Eigen::Matrix<double, ErrorDim, BlockDim>;
  template <int BlockDim>
  using BlockVector =
      std::vector<BlockMatrix<BlockDim>,
                  Eigen::aligned_allocator<BlockMatrix<BlockDim>>>;
  using Blocks = std::tuple<BlockVector<BlockDims>...>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  std::vector<size_t> keyDims_;
  std::vector<SlotIndices> rowSlots_;
  Blocks blocks_;
  std::vector<RhsVector, Eigen::aligned_allocator<RhsVector>> rhs_;
  SharedDiagonal model_;

  template <size_t... Indices>
  void reserveBlocks(size_t rowCount, std::index_sequence<Indices...>) {
    (std::get<Indices>(blocks_).reserve(rowCount), ...);
  }

  template <size_t Slot>
  void addBlock(const Matrix& block) {
    using BlockVectorType = typename std::tuple_element<Slot, Blocks>::type;
    using BlockType = typename BlockVectorType::value_type;
    constexpr int BlockDim = BlockType::ColsAtCompileTime;
    if (block.rows() != ErrorDim || block.cols() != BlockDim) {
      throw std::invalid_argument(
          "BatchJacobianFactor::addRow: incompatible block dimension.");
    }
    std::get<Slot>(blocks_).push_back(block);
  }

  template <size_t... Indices>
  void addBlocks(const std::vector<Matrix>& blocks,
                 std::index_sequence<Indices...>) {
    (addBlock<Indices>(blocks[Indices]), ...);
  }

  template <size_t Slot>
  void copyBlockToDense(size_t rowIndex, VerticalBlockMatrix* dense) const {
    using BlockVectorType = typename std::tuple_element<Slot, Blocks>::type;
    using BlockType = typename BlockVectorType::value_type;
    constexpr int BlockDim = BlockType::ColsAtCompileTime;
    const DenseIndex keySlot = rowSlots_[rowIndex][Slot];
    (*dense)(keySlot).block(static_cast<DenseIndex>(rowIndex * ErrorDim), 0,
                            ErrorDim, BlockDim) =
        std::get<Slot>(blocks_)[rowIndex];
  }

  template <size_t... Indices>
  void copyBlocksToDense(size_t rowIndex, VerticalBlockMatrix* dense,
                         std::index_sequence<Indices...>) const {
    (copyBlockToDense<Indices>(rowIndex, dense), ...);
  }

  template <size_t Slot>
  void scatterBlock(size_t rowIndex, size_t rowOffset,
                    VerticalBlockMatrix* target,
                    const std::vector<DenseIndex>& targetBlockIndices) const {
    using BlockVectorType = typename std::tuple_element<Slot, Blocks>::type;
    using BlockType = typename BlockVectorType::value_type;
    constexpr int BlockDim = BlockType::ColsAtCompileTime;
    const DenseIndex keySlot = rowSlots_[rowIndex][Slot];
    const DenseIndex targetBlock = targetBlockIndices[keySlot];
    if (targetBlock < 0) return;
    (*target)(targetBlock)
        .block(static_cast<DenseIndex>(rowOffset + rowIndex * ErrorDim), 0,
               ErrorDim, BlockDim) = std::get<Slot>(blocks_)[rowIndex];
  }

  template <size_t... Indices>
  void scatterBlocks(size_t rowIndex, size_t rowOffset,
                     VerticalBlockMatrix* target,
                     const std::vector<DenseIndex>& targetBlockIndices,
                     std::index_sequence<Indices...>) const {
    (scatterBlock<Indices>(rowIndex, rowOffset, target, targetBlockIndices),
     ...);
  }

 public:
  /// Construct an empty compact batch factor with known key dimensions.
  BatchJacobianFactor(const KeyVector& keys, std::vector<size_t> keyDims,
                      const SharedDiagonal& model = SharedDiagonal())
      : Base(keys), keyDims_(std::move(keyDims)), model_(model) {
    if (keyDims_.size() != keys_.size()) {
      throw std::invalid_argument(
          "BatchJacobianFactor: key dimension count must match keys.");
    }
  }

  GaussianFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<GaussianFactor>(
        std::make_shared<This>(*this));
  }

  /// Reserve row-group storage.
  void reserve(size_t rowCount) {
    rowSlots_.reserve(rowCount);
    rhs_.reserve(rowCount);
    reserveBlocks(rowCount, std::make_index_sequence<NumSlots>{});
  }

  /// Add one row group corresponding to one original nonlinear factor.
  void addRow(const SlotIndices& slots, const std::vector<Matrix>& blocks,
              const Vector& rhs) {
    if (blocks.size() != NumSlots || rhs.size() != ErrorDim) {
      throw std::invalid_argument(
          "BatchJacobianFactor::addRow: incompatible row dimensions.");
    }
    rowSlots_.push_back(slots);
    addBlocks(blocks, std::make_index_sequence<NumSlots>{});
    RhsVector fixedRhs = rhs;
    rhs_.push_back(fixedRhs);
  }

  size_t rows() const override { return rhs_.size() * ErrorDim; }

  const SharedDiagonal& get_model() const override { return model_; }

  DenseIndex getDim(const_iterator variable) const override {
    return static_cast<DenseIndex>(keyDims_.at(variable - begin()));
  }

  /// Return the compact key slot used by each row group and factor slot.
  const std::vector<SlotIndices>& rowSlots() const { return rowSlots_; }

  /// Convert compact row-block storage into a conventional JacobianFactor.
  JacobianFactor toJacobianFactor() const override {
    if (rowSlots_.empty()) return JacobianFactor();
    VerticalBlockMatrix dense(keyDims_, static_cast<DenseIndex>(rows()), true);
    dense.matrix().setZero();
    for (size_t rowIndex = 0; rowIndex < rowSlots_.size(); ++rowIndex) {
      copyBlocksToDense(rowIndex, &dense, std::make_index_sequence<NumSlots>{});
      dense(keys_.size())
          .block(static_cast<DenseIndex>(rowIndex * ErrorDim), 0, ErrorDim, 1) =
          rhs_[rowIndex];
    }
    return JacobianFactor(keys_, std::move(dense), model_);
  }

  size_t scatterInto(
      VerticalBlockMatrix& target, size_t rowOffset,
      const std::vector<DenseIndex>& targetBlockIndices) const override {
    if (targetBlockIndices.size() != keys_.size()) {
      throw std::invalid_argument(
          "BatchJacobianFactor::scatterInto: target index count mismatch.");
    }
    const size_t rhsBlock = target.nBlocks() - 1;
    for (size_t rowIndex = 0; rowIndex < rowSlots_.size(); ++rowIndex) {
      scatterBlocks(rowIndex, rowOffset, &target, targetBlockIndices,
                    std::make_index_sequence<NumSlots>{});
      target(rhsBlock).block(
          static_cast<DenseIndex>(rowOffset + rowIndex * ErrorDim), 0, ErrorDim,
          1) = rhs_[rowIndex];
    }
    return rows();
  }
};

}  // namespace gtsam
