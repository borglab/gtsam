/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HostSparseJacobian.h
 * @brief   Pinned host staging for a planned sparse Jacobian and right-hand side
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#pragma once

#include <gtsam/base/cuda/PinnedHostArray.h>
#include <gtsam/nonlinear/cuda/internal/SparseJacobianPlan.h>

#include <cstddef>
#include <cstdint>

namespace gtsam::cuda {

/**
 * Pinned host staging buffer holding one linearization's Jacobian values and
 * right-hand side in the exact layout a SparseJacobianPlan prescribes.
 *
 * This exists because GTSAM's own linearization output is the wrong shape to
 * upload. NonlinearFactorGraph::linearize() returns a GaussianFactorGraph of
 * JacobianFactors, each owning a separate heap-allocated VerticalBlockMatrix, so
 * the numbers for one iteration are scattered across thousands of allocations at
 * addresses that change every iteration. Copying that to the device means either
 * thousands of small transfers or repacking into a contiguous buffer first, and
 * the buffer has to be pinned for the transfer to be fast and asynchronous.
 *
 * So the split is: SparseJacobianPlan holds the structure — CSR row pointers,
 * column indices, and the per-factor recipe saying where each block writes —
 * computed once and uploaded once. This holds only the numbers, in the flat
 * order that structure implies, and is overwritten in place on every
 * linearization. That is why clear() zeroes rather than reallocating: the
 * addresses must stay fixed so an upload can be queued against them repeatedly.
 * The CSR pattern deliberately does not appear here; only values and rhs do.
 *
 * The plan's structural fingerprint is copied at construction so a buffer can be
 * checked against the plan it was sized for rather than silently accepting a
 * same-sized one from a different graph. Sizes come from the plan and never
 * change; a new plan needs a new buffer.
 */
class HostSparseJacobian {
 public:
  /// Allocates zeroed pinned storage sized by the plan, which is not retained.
  explicit HostSparseJacobian(const SparseJacobianPlan& plan)
      : structuralFingerprint_(plan.structuralFingerprint()),
        values_(static_cast<size_t>(plan.nonzeros())),
        rhs_(static_cast<size_t>(plan.rows())) {
    clear();
  }

  /// Zeroes both buffers in place, leaving their addresses and sizes unchanged.
  void clear() {
    values_.clear();
    rhs_.clear();
  }

  /// Borrows the flat Jacobian values, in the plan's CSR order.
  double* valuesData() { return values_.data(); }
  /// Borrows the flat Jacobian values, in the plan's CSR order.
  const double* valuesData() const { return values_.data(); }
  /// Borrows the right-hand side b of Jx = b, one entry per scalar row.
  double* rhsData() { return rhs_.data(); }
  /// Borrows the right-hand side b of Jx = b, one entry per scalar row.
  const double* rhsData() const { return rhs_.data(); }
  /// Returns the stored nonzero count, which equals the plan's.
  size_t valuesSize() const { return values_.size(); }
  /// Returns the scalar row count, which equals the plan's.
  size_t rhsSize() const { return rhs_.size(); }
  /// Returns the fingerprint of the plan this buffer was sized for.
  uint64_t structuralFingerprint() const { return structuralFingerprint_; }

 private:
  uint64_t structuralFingerprint_ = 0;
  PinnedHostArray<double> values_;
  PinnedHostArray<double> rhs_;
};

}  // namespace gtsam::cuda
