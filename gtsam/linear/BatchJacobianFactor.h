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
#include <gtsam/base/timing.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>

#include <Eigen/StdVector>
#include <algorithm>
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
 * BatchJacobianFactorBase gives solvers a way to recognize compact batched
 * Jacobian storage without treating it as a regular dense-block
 * JacobianFactor. Generic GaussianFactor methods keep working through
 * toJacobianFactor(), while solvers that know this type can use scatterInto()
 * or derived-class updateHessian() implementations to avoid materializing
 * structural zeros.
 *
 * The intended fast path is a factor whose row blocks have already been
 * whitened and therefore has either no model or a unit model. Non-unit diagonal
 * models are still supported for compatibility paths.
 */
class GTSAM_EXPORT BatchJacobianFactorBase : public GaussianFactor {
 public:
  using GaussianFactor::GaussianFactor;

  /// Number of scalar rows represented by this factor.
  virtual size_t rows() const = 0;

  /// Optional model on the compact factor. The intended fast path is whitened.
  virtual const SharedDiagonal& get_model() const = 0;

  /// Convert compact storage to a dense-block JacobianFactor.
  virtual JacobianFactor toJacobianFactor() const = 0;

  /**
   * Scatter nonzero row blocks into an existing vertical block matrix.
   *
   * This is the QR/new-multifrontal fast path. The destination matrix is already
   * allocated for a clique, and this method writes only the blocks present in
   * each compact row group. Dense blocks implied by absent key slots are left
   * untouched and are expected to have been zero-initialized by the caller.
   *
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

  /**
   * Update augmented Hessian over all local columns using precomputed
   * clique-local block indices.
   *
   * Callers in `MultifrontalClique` can use this to avoid repeated key lookup
   * when a precomputed load plan already maps factor slots to clique blocks.
   */
  virtual void updateHessian(const std::vector<DenseIndex>& slotIndices,
                            SymmetricBlockMatrix* info) const = 0;

  /**
   * Update an inclusive column-range of the augmented Hessian using precomputed
   * clique-local block indices.
   */
  virtual void updateHessian(const std::vector<DenseIndex>& slotIndices,
                            SymmetricBlockMatrix* info, DenseIndex beginCol,
                            DenseIndex endCol) const = 0;

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
 * This factor stores many small Jacobian row groups in compact form. A row
 * group corresponds to one original nonlinear factor and has ErrorDim scalar
 * rows. For each row group we store:
 *
 * - the local key slot used by each factor variable,
 * - one fixed-size Jacobian block for each factor variable, and
 * - one fixed-size right-hand side vector.
 *
 * The factor's key list is the union of all keys appearing in the batch. Unlike
 * a dense JacobianFactor, a row group only stores blocks for the keys touched by
 * that original factor. For example, point-first SFM batches avoid storing
 * zero camera blocks for cameras that did not observe the point.
 *
 * The template parameters require fixed dimensions so the hot loops can use
 * Eigen fixed-size products. Dynamic-dimension factors should use the ordinary
 * JacobianFactor fallback in BatchFactor.
 *
 * Noise-model convention: if model_ is null or unit, blocks and right-hand sides
 * are treated as already whitened. If model_ is a non-unit diagonal model,
 * updateHessian() applies the per-row diagonal weights directly. Constrained
 * models are intentionally rejected by updateHessian(), matching
 * JacobianFactor's Hessian assembly behavior.
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
  using HessianSlots = std::array<DenseIndex, NumSlots + 1>;
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

  /// Return whether a block slot lies in the requested Hessian column range.
  static bool slotInRange(DenseIndex slot, DenseIndex beginCol,
                          DenseIndex endCol) {
    return slot >= beginCol && slot < endCol;
  }

  /**
   * Add one diagonal block contribution from a compact row group.
   *
   * Slot == NumSlots denotes the augmented right-hand side block. Otherwise Slot
   * selects one Jacobian block in the row group. If weights is non-null, this
   * computes A'WA; otherwise it computes A'A.
   */
  template <size_t Slot>
  void updateAugmentedDiagonal(size_t rowIndex, DenseIndex targetSlot,
                               const RhsVector* weights,
                               SymmetricBlockMatrix* info) const {
    if constexpr (Slot == NumSlots) {
      const RhsVector& b = rhs_[rowIndex];
      if (weights) {
        info->updateDiagonalBlock(
            targetSlot, b.transpose() * weights->asDiagonal() * b);
      } else {
        info->diagonalBlock(targetSlot).rankUpdate(b.transpose());
      }
    } else {
      const auto& A = std::get<Slot>(blocks_)[rowIndex];
      if (weights) {
        info->updateDiagonalBlock(
            targetSlot, A.transpose() * weights->asDiagonal() * A);
      } else {
        info->diagonalBlock(targetSlot).rankUpdate(A.transpose());
      }
    }
  }

  /**
   * Add one off-diagonal augmented Hessian block from a compact row group.
   *
   * J == NumSlots denotes the right-hand side block, producing A'b. Otherwise
   * this computes Ai'Aj for two Jacobian blocks. Non-null weights apply the
   * diagonal model as Ai'WAj.
   */
  template <size_t I, size_t J>
  void updateAugmentedOffDiagonal(size_t rowIndex, DenseIndex targetI,
                                  DenseIndex targetJ,
                                  const RhsVector* weights,
                                  SymmetricBlockMatrix* info) const {
    static_assert(I < J, "BatchJacobianFactor expects upper-triangular order.");
    if constexpr (J == NumSlots) {
      const auto& A = std::get<I>(blocks_)[rowIndex];
      const RhsVector& b = rhs_[rowIndex];
      if (weights) {
        info->updateOffDiagonalBlock(
            targetI, targetJ, A.transpose() * weights->asDiagonal() * b);
      } else {
        info->updateOffDiagonalBlock(targetI, targetJ, A.transpose() * b);
      }
    } else {
      const auto& Ai = std::get<I>(blocks_)[rowIndex];
      const auto& Aj = std::get<J>(blocks_)[rowIndex];
      if (weights) {
        info->updateOffDiagonalBlock(
            targetI, targetJ, Ai.transpose() * weights->asDiagonal() * Aj);
      } else {
        info->updateOffDiagonalBlock(targetI, targetJ, Ai.transpose() * Aj);
      }
    }
  }

  /// Update all previous slots that contribute to augmented column J.
  template <size_t J, size_t... Is>
  void updatePreviousAugmentedSlots(
      size_t rowIndex, const HessianSlots& slots, const RhsVector* weights,
      SymmetricBlockMatrix* info, DenseIndex beginCol, DenseIndex endCol,
      std::index_sequence<Is...>) const {
    ((slotInRange(std::max(slots[Is], slots[J]), beginCol, endCol)
          ? updateAugmentedOffDiagonal<Is, J>(rowIndex, slots[Is], slots[J],
                                              weights, info)
          : void()),
     ...);
  }

  /// Update augmented column J for one compact row group.
  template <size_t J>
  void updateAugmentedColumn(size_t rowIndex, const HessianSlots& slots,
                             const RhsVector* weights,
                             SymmetricBlockMatrix* info, DenseIndex beginCol,
                             DenseIndex endCol) const {
    const DenseIndex targetSlot = slots[J];
    if (slotInRange(targetSlot, beginCol, endCol)) {
      updateAugmentedDiagonal<J>(rowIndex, targetSlot, weights, info);
    }
    updatePreviousAugmentedSlots<J>(rowIndex, slots, weights, info, beginCol,
                                    endCol, std::make_index_sequence<J>{});
  }

  /// Unroll all augmented-column updates for one compact row group.
  template <size_t... Js>
  void updateAugmentedColumns(size_t rowIndex, const HessianSlots& slots,
                              const RhsVector* weights,
                              SymmetricBlockMatrix* info, DenseIndex beginCol,
                              DenseIndex endCol,
                              std::index_sequence<Js...>) const {
    (updateAugmentedColumn<Js>(rowIndex, slots, weights, info, beginCol,
                               endCol),
     ...);
  }

  /**
   * Add all Hessian contributions for one compact row group.
   *
   * The passed slots vector maps this factor's global key slots into the target
   * SymmetricBlockMatrix. The row's SlotIndices then select the active subset
   * used by this row group.
   */
  void updateHessianRow(size_t rowIndex, const std::vector<DenseIndex>& slots,
                        SymmetricBlockMatrix* info, DenseIndex beginCol,
                        DenseIndex endCol) const {
    HessianSlots rowSlots;
    for (size_t j = 0; j < NumSlots; ++j) {
      rowSlots[j] = slots[rowSlots_[rowIndex][j]];
    }
    rowSlots[NumSlots] = slots[keys_.size()];

    RhsVector weights;
    const RhsVector* weightsPtr = nullptr;
    if (model_ && !model_->isUnit()) {
      weights = model_->invsigmas()
                    .template segment<ErrorDim>(
                        static_cast<DenseIndex>(rowIndex * ErrorDim))
                    .array()
                    .square();
      weightsPtr = &weights;
    }

    updateAugmentedColumns(rowIndex, rowSlots, weightsPtr, info, beginCol,
                           endCol,
                           std::make_index_sequence<NumSlots + 1>{});
  }

 public:
  /**
   * Construct an empty compact batch factor with known key dimensions.
   *
   * @param keys Union of all keys represented by this batch factor.
   * @param keyDims Dimensions corresponding to keys, in the same order.
   * @param model Optional diagonal model on the stored rows.
   */
  BatchJacobianFactor(const KeyVector& keys, std::vector<size_t> keyDims,
                      const SharedDiagonal& model = SharedDiagonal())
      : Base(keys), keyDims_(std::move(keyDims)), model_(model) {
    if (keyDims_.size() != keys_.size()) {
      throw std::invalid_argument(
          "BatchJacobianFactor: key dimension count must match keys.");
    }
  }

  /// Return a deep copy as a GaussianFactor.
  GaussianFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<GaussianFactor>(
        std::make_shared<This>(*this));
  }

  /// Reserve storage for row groups before repeated addRow() calls.
  void reserve(size_t rowCount) {
    rowSlots_.reserve(rowCount);
    rhs_.reserve(rowCount);
    reserveBlocks(rowCount, std::make_index_sequence<NumSlots>{});
  }

  /**
   * Add one row group corresponding to one original nonlinear factor.
   *
   * @param slots Local key slots touched by this row group. Each entry indexes
   *        into this factor's keys() vector.
   * @param blocks Fixed-size Jacobian blocks, one for each factor variable.
   * @param rhs Right-hand side vector for this row group.
   */
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

  /// Return the number of scalar rows represented by all row groups.
  size_t rows() const override { return rhs_.size() * ErrorDim; }

  /// Return the optional diagonal model on the stored rows.
  const SharedDiagonal& get_model() const override { return model_; }

  /// Return the dimension of the variable at the given key iterator.
  DenseIndex getDim(const_iterator variable) const override {
    return static_cast<DenseIndex>(keyDims_.at(variable - begin()));
  }

  /// Return the compact key slot used by each row group and factor slot.
  const std::vector<SlotIndices>& rowSlots() const { return rowSlots_; }

  /**
   * Convert compact row-block storage into a conventional JacobianFactor.
   *
   * This compatibility path allocates dense blocks for every key in the batch,
   * including structural zeros. Performance-critical solvers should prefer
   * scatterInto() or updateHessian().
   */
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

  /**
   * Scatter this factor into a preallocated vertical block matrix.
   *
   * Only the row group's active key blocks and right-hand side are written.
   */
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

  /**
   * Update this factor's augmented information using precomputed clique-local
   * block indices.
   *
   * This avoids key lookups in repeated precomputed-load passes.
   */
  void updateHessian(const std::vector<DenseIndex>& slotIndices,
                     SymmetricBlockMatrix* info) const override {
    updateHessian(slotIndices, info, 0, info->nBlocks());
  }

  /**
   * Update this factor's augmented information over a column slice.
   *
   * The `slotIndices` argument maps each factor key slot to the destination
   * SymmetricBlockMatrix block, or to -1 when the key is fixed to zero.
   */
  void updateHessian(const std::vector<DenseIndex>& slotIndices,
                     SymmetricBlockMatrix* info, DenseIndex beginCol,
                     DenseIndex endCol) const override {
    gttic(updateHessian_BatchJacobianFactor);
    if (rows() == 0) return;
    if (slotIndices.size() != keys_.size()) {
      throw std::invalid_argument(
          "BatchJacobianFactor::updateHessian: slot index count mismatch.");
    }
    if (model_ && !model_->isUnit() && model_->isConstrained()) {
      throw std::invalid_argument(
          "BatchJacobianFactor::updateHessian: cannot update information with "
          "constrained noise model");
    }

    std::vector<DenseIndex> slots;
    slots.reserve(slotIndices.size() + 1);
    bool foundCol = false;
    for (const DenseIndex slot : slotIndices) {
      slots.push_back(slot);
      if (slotInRange(slot, beginCol, endCol)) foundCol = true;
    }
    slots.push_back(info->nBlocks() - 1);
    if (slotInRange(slots.back(), beginCol, endCol)) foundCol = true;
    if (!foundCol) return;

    for (size_t rowIndex = 0; rowIndex < rowSlots_.size(); ++rowIndex) {
      updateHessianRow(rowIndex, slots, info, beginCol, endCol);
    }
  }

  /**
   * Add this factor's augmented information matrix to info.
   *
   * This direct implementation is the legacy Cholesky fast path. It accumulates
   * fixed-size row-group products into the target SymmetricBlockMatrix and avoids
   * constructing a dense compatibility JacobianFactor.
   */
  void updateHessian(const KeyVector& infoKeys,
                     SymmetricBlockMatrix* info) const override {
    updateHessian(infoKeys, info, 0, info->nBlocks());
  }

  /**
   * Add this factor's augmented information matrix over a block-column range.
   *
   * This overload supports the partial-column Hessian assembly used by parallel
   * Cholesky paths. It follows the same column ownership convention as
   * JacobianFactor: a block (I,J) is updated when max(I,J) lies in
   * [beginCol,endCol).
   */
  void updateHessian(const KeyVector& infoKeys, SymmetricBlockMatrix* info,
                     DenseIndex beginCol, DenseIndex endCol) const override {
    gttic(updateHessian_BatchJacobianFactor);
    if (rows() == 0) return;
    if (model_ && !model_->isUnit() && model_->isConstrained()) {
      throw std::invalid_argument(
          "BatchJacobianFactor::updateHessian: cannot update information with "
          "constrained noise model");
    }

    std::vector<DenseIndex> slots;
    slots.reserve(keys_.size() + 1);
    bool foundCol = false;
    for (Key key : keys_) {
      const DenseIndex slot = Slot(infoKeys, key);
      slots.push_back(slot);
      if (slotInRange(slot, beginCol, endCol)) foundCol = true;
    }
    slots.push_back(info->nBlocks() - 1);
    if (slotInRange(slots.back(), beginCol, endCol)) foundCol = true;
    if (!foundCol) return;

    for (size_t rowIndex = 0; rowIndex < rowSlots_.size(); ++rowIndex) {
      updateHessianRow(rowIndex, slots, info, beginCol, endCol);
    }
  }
};

}  // namespace gtsam
