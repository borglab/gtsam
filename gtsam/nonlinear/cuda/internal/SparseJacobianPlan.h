/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SparseJacobianPlan.h
 * @brief   Symbolic plan mapping a factor graph onto a fixed sparse Jacobian
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/KeyInfo.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace gtsam::cuda {

/// Describes where one factor key writes within every CSR row it owns.
struct SparseJacobianBlockWritePlan {
  /// Factor key associated with this block.
  Key key = 0;
  /// Position of the key in the factor's local key list.
  size_t localBlockIndex = 0;
  /// Scalar width of the block.
  int width = 0;
  /// First scalar column in the global flattened layout.
  int globalColumnBegin = 0;
  /// First value slot for this block relative to one factor row.
  int valueOffsetWithinRow = 0;
};

/// Host write recipe and row range for one graph factor.
struct SparseJacobianFactorWritePlan {
  /// First global scalar row owned by the factor.
  int rowBegin = 0;
  /// Number of scalar residual rows.
  int rowCount = 0;
  /// Number of stored Jacobian values in each residual row.
  int nonzerosPerRow = 0;
  /// Whether the factor supports direct device transfer.
  bool sendable = true;
  /// Per-key block write recipes in factor-local key order.
  std::vector<SparseJacobianBlockWritePlan> blocks;
};

/**
 * Immutable host-side CSR structure and factor write plan for a nonlinear
 * graph. Returned references borrow this plan and remain valid until it is
 * moved or destroyed. The plan owns no graph, factor, or Values objects.
 */
class GTSAM_EXPORT SparseJacobianPlan {
 public:
  /// Compiles graph structure against columns and validates full coverage.
  SparseJacobianPlan(const NonlinearFactorGraph& graph,
                     const KeyInfo& columns);

  /// Returns the total residual row count.
  int rows() const;
  /// Returns the total flattened variable dimension.
  int columns() const;
  /// Returns the number of structurally stored Jacobian values.
  int nonzeros() const;
  /// Borrows the zero-based int32 CSR row pointers.
  const std::vector<int>& rowPointers() const;
  /// Borrows the zero-based int32 CSR column indices.
  const std::vector<int>& columnIndices() const;
  /// Returns the write plan for a graph index or throws std::out_of_range.
  const SparseJacobianFactorWritePlan& factor(size_t index) const;
  /// Borrows all per-factor write plans in graph order.
  const std::vector<SparseJacobianFactorWritePlan>& factors() const;
  /// Returns the stored graph-and-layout structural fingerprint.
  uint64_t structuralFingerprint() const;
  /// Checks exact structural compatibility without retaining either input.
  bool matches(const NonlinearFactorGraph& graph,
               const KeyInfo& columns) const;

 private:
  int rows_ = 0;
  int columns_ = 0;
  std::vector<int> rowPointers_{0};
  std::vector<int> columnIndices_;
  std::vector<SparseJacobianFactorWritePlan> factors_;
  uint64_t structuralFingerprint_ = 0;

  KeyInfo columnInfo_;
  std::vector<bool> factorIsNull_;
};

}  // namespace gtsam::cuda
