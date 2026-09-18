/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BatchJacobianFactorElimination.h
 * @brief Internal solver access to compact batch Jacobian factors.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/internal/BatchHessianMapping.h>

#include <stdexcept>
#include <vector>

namespace gtsam::internal {

/**
 * Solver-facing adapter for compact batch-factor storage.
 *
 * This class keeps row-group mapping and direct-assembly details out of the
 * public `BatchJacobianFactorBase` API. It is the only friend allowed to call
 * those private virtual hooks, while numerical implementations remain in the
 * header-only fixed-dimension factor template.
 */
class BatchJacobianFactorElimination {
 public:
  /** Build row-group block destinations from factor-key destinations. */
  static BatchHessianMapping buildMapping(
      const BatchJacobianFactorBase& factor,
      const std::vector<DenseIndex>& factorBlockSlots) {
    BatchHessianMapping mapping;
    factor.buildMappedSlots(factorBlockSlots, mapping.blockSlots);
    return mapping;
  }

  /**
   * Build row-group block and scalar destinations from paired factor-key
   * destinations.
   */
  static BatchHessianMapping buildMapping(
      const BatchJacobianFactorBase& factor,
      const std::vector<DenseIndex>& factorBlockSlots,
      const std::vector<DenseIndex>& factorScalarOffsets) {
    if (factorBlockSlots.size() != factorScalarOffsets.size()) {
      throw std::invalid_argument(
          "BatchJacobianFactorElimination::buildMapping: destination count "
          "mismatch.");
    }
    BatchHessianMapping mapping;
    factor.buildMappedSlots(factorBlockSlots, mapping.blockSlots);
    factor.buildMappedSlots(factorScalarOffsets, mapping.scalarOffsets);
    return mapping;
  }

  /** Cache scalar destinations for an existing block mapping. */
  static void cacheScalarOffsets(const SymmetricBlockMatrix& target,
                                 BatchHessianMapping* mapping) {
    if (!mapping) {
      throw std::invalid_argument(
          "BatchJacobianFactorElimination::cacheScalarOffsets: null mapping.");
    }
    mapping->scalarOffsets.clear();
    mapping->scalarOffsets.reserve(mapping->blockSlots.size());
    for (const DenseIndex slot : mapping->blockSlots) {
      mapping->scalarOffsets.push_back(
          slot < 0 ? -1 : target.blockScalarOffset(slot));
    }
  }

  /** Scatter compact Jacobian rows into preallocated clique storage. */
  static size_t scatterRows(const BatchJacobianFactorBase& factor,
                            VerticalBlockMatrix& target, size_t rowOffset,
                            const std::vector<DenseIndex>& targetBlockIndices) {
    return factor.scatterInto(target, rowOffset, targetBlockIndices);
  }

  /** Add compact rows directly to a sparse normal-equation accumulator. */
  static void addSparseNormal(const BatchJacobianFactorBase& factor,
                              const std::vector<DenseIndex>& scalarOffsets,
                              SparseNormalAccumulator* accumulator) {
    factor.updateSparseNormal(scalarOffsets, accumulator);
  }

  /** Add the augmented Hessian using factor-key block destinations. */
  static void addHessian(const BatchJacobianFactorBase& factor,
                         const std::vector<DenseIndex>& factorBlockSlots,
                         SymmetricBlockMatrix* target) {
    factor.updateHessian(factorBlockSlots, target);
  }

  /** Add a block-column range using factor-key block destinations. */
  static void addHessianColumns(const BatchJacobianFactorBase& factor,
                                const std::vector<DenseIndex>& factorBlockSlots,
                                SymmetricBlockMatrix* target,
                                DenseIndex beginColumn, DenseIndex endColumn) {
    factor.updateHessian(factorBlockSlots, target, beginColumn, endColumn);
  }

  /** Add the augmented Hessian using cached row-group destinations. */
  static void addHessian(const BatchJacobianFactorBase& factor,
                         const BatchHessianMapping& mapping,
                         SymmetricBlockMatrix* target) {
    if (mapping.scalarOffsets.empty()) {
      factor.updateHessianWithMappedSlots(mapping.blockSlots, target);
    } else {
      if (!mapping.hasScalarOffsets()) {
        throw std::invalid_argument(
            "BatchJacobianFactorElimination::addHessian: scalar destination "
            "count mismatch.");
      }
      factor.updateHessianWithMappedSlots(mapping.blockSlots,
                                          mapping.scalarOffsets, target);
    }
  }

  /** Add frontal Hessian rows using cached row-group destinations. */
  static void addFrontalHessian(const BatchJacobianFactorBase& factor,
                                const BatchHessianMapping& mapping,
                                DenseIndex numFrontalBlocks,
                                VerticalBlockMatrix* frontalRows) {
    factor.updateFrontalHessianWithMappedSlots(mapping.blockSlots,
                                               numFrontalBlocks, frontalRows);
  }
};

}  // namespace gtsam::internal
