/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SparseJacobianPlan.cpp
 * @brief   Symbolic plan mapping a factor graph onto a fixed sparse Jacobian
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#include <gtsam/base/types.h>
#include <gtsam/linear/cuda/internal/BlockOrdering.h>
#include <gtsam/nonlinear/cuda/internal/SparseJacobianPlan.h>

#ifdef GTSAM_USE_TBB
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace gtsam::cuda {
namespace {

constexpr uint64_t kFnv1aOffsetBasis = 14695981039346656037ULL;
constexpr uint64_t kFnv1aPrime = 1099511628211ULL;

int checkedSizeToInt(size_t value, const char* description) {
  if (value > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(std::string(description) +
                                " exceeds signed int capacity");
  }
  return static_cast<int>(value);
}

int checkedAdd(int left, int right, const char* description) {
  if (left < 0 || right < 0 || right > std::numeric_limits<int>::max() - left) {
    throw std::invalid_argument(std::string(description) +
                                " exceeds signed int capacity");
  }
  return left + right;
}

int checkedMultiply(int left, int right, const char* description) {
  if (left < 0 || right < 0 ||
      (left != 0 && right > std::numeric_limits<int>::max() / left)) {
    throw std::invalid_argument(std::string(description) +
                                " exceeds signed int capacity");
  }
  return left * right;
}

const KeyInfoEntry& findColumnOrThrow(const KeyInfo& columns, Key key) {
  try {
    return columns.at(key);
  } catch (const std::out_of_range&) {
    throw std::invalid_argument("SparseJacobianPlan factor key " +
                                DefaultKeyFormatter(key) +
                                " is missing from the column layout");
  }
}

class Fnv1a64 {
 public:
  void appendByte(uint8_t value) {
    value_ ^= value;
    value_ *= kFnv1aPrime;
  }

  void appendUint64(uint64_t value) {
    for (int byte = 0; byte < 8; ++byte) {
      appendByte(static_cast<uint8_t>(value & 0xffU));
      value >>= 8;
    }
  }

  void appendInt(int value) { appendUint64(static_cast<uint64_t>(value)); }

  void appendSize(size_t value) {
    if constexpr (sizeof(size_t) > sizeof(uint64_t)) {
      if (value > std::numeric_limits<uint64_t>::max()) {
        throw std::invalid_argument("structure size exceeds uint64 capacity");
      }
    }
    appendUint64(static_cast<uint64_t>(value));
  }

  void appendBool(bool value) { appendByte(value ? 1U : 0U); }

  uint64_t value() const { return value_; }

 private:
  uint64_t value_ = kFnv1aOffsetBasis;
};

void appendColumns(Fnv1a64* hash, const KeyInfo& keyInfo) {
  hash->appendSize(keyInfo.size());
  hash->appendSize(keyInfo.numCols());
  for (const Key key : keyInfo.ordering()) {
    const KeyInfoEntry& entry = keyInfo.at(key);
    hash->appendUint64(static_cast<uint64_t>(key));
    hash->appendSize(entry.index);
    hash->appendSize(entry.dim);
    hash->appendSize(entry.start);
  }
}

uint64_t computeStoredFingerprint(
    const KeyInfo& columnInfo,
    const std::vector<SparseJacobianFactorWritePlan>& factors,
    const std::vector<bool>& factorIsNull) {
  Fnv1a64 hash;
  appendColumns(&hash, columnInfo);
  hash.appendSize(factors.size());

  for (size_t index = 0; index < factors.size(); ++index) {
    const SparseJacobianFactorWritePlan& factor = factors[index];
    hash.appendBool(factorIsNull.at(index));
    hash.appendInt(factor.rowCount);
    hash.appendSize(factor.blocks.size());
    for (const SparseJacobianBlockWritePlan& block : factor.blocks) {
      hash.appendUint64(static_cast<uint64_t>(block.key));
      hash.appendInt(block.width);
    }
    hash.appendBool(factor.sendable);
  }
  return hash.value();
}

uint64_t computeGraphFingerprint(const NonlinearFactorGraph& graph,
                                 const KeyInfo& columns) {
  Fnv1a64 hash;
  appendColumns(&hash, columns);
  hash.appendSize(graph.size());

  for (const NonlinearFactor::shared_ptr& factor : graph) {
    const bool isNull = !factor;
    hash.appendBool(isNull);
    if (isNull) {
      hash.appendInt(0);
      hash.appendSize(0);
      hash.appendBool(true);
      continue;
    }

    hash.appendInt(checkedSizeToInt(factor->dim(), "factor row count"));
    hash.appendSize(factor->keys().size());
    for (Key key : factor->keys()) {
      const KeyInfoEntry& column = findColumnOrThrow(columns, key);
      hash.appendUint64(static_cast<uint64_t>(key));
      hash.appendSize(column.dim);
    }
    hash.appendBool(factor->sendable());
  }
  return hash.value();
}

bool sameKeyInfo(const KeyInfo& expected, const KeyInfo& actual) {
  if (expected.size() != actual.size()) {
    return false;
  }
  if (expected.ordering() != actual.ordering() ||
      expected.numCols() != actual.numCols()) {
    return false;
  }
  for (const Key key : expected.ordering()) {
    const auto found = actual.find(key);
    if (found == actual.end()) {
      return false;
    }
    const KeyInfoEntry& expectedEntry = expected.at(key);
    const KeyInfoEntry& actualEntry = found->second;
    if (expectedEntry.index != actualEntry.index ||
        expectedEntry.dim != actualEntry.dim ||
        expectedEntry.start != actualEntry.start) {
      return false;
    }
  }
  return true;
}

}  // namespace

SparseJacobianPlan::SparseJacobianPlan(
    const NonlinearFactorGraph& graph, const KeyInfo& columns)
    : columns_(cudaBlockOffsets(columns).back()), columnInfo_(columns) {
  const size_t factorCount = graph.size();
  factors_.resize(factorCount);
  factorIsNull_.resize(factorCount);

  // Phase A (parallel over factors): build each factor's write plan.
  // Everything here depends only on that factor and the read-only column
  // layout; rowBegin and valueOffsetWithinRow are per-factor-local. Overflow
  // of per-factor quantities is checked here; graph-cumulative overflow is
  // checked in the serial prefix phase.
  const auto buildFactorPlan = [&](size_t index) {
    const NonlinearFactor::shared_ptr& factor = graph[index];
    const bool isNull = !factor;
    factorIsNull_[index] = isNull;
    if (isNull) return;

    SparseJacobianFactorWritePlan& factorPlan = factors_[index];
    factorPlan.rowCount = checkedSizeToInt(factor->dim(), "factor row count");
    factorPlan.sendable = factor->sendable();

    const KeyVector& keys = factor->keys();
    factorPlan.blocks.reserve(keys.size());
    for (size_t localBlockIndex = 0; localBlockIndex < keys.size();
         ++localBlockIndex) {
      const Key key = keys[localBlockIndex];
      for (size_t earlier = 0; earlier < localBlockIndex; ++earlier) {
        if (keys[earlier] == key) {
          throw std::invalid_argument(
              "SparseJacobianPlan repeated factor key " +
              DefaultKeyFormatter(key));
        }
      }

      const KeyInfoEntry& column = findColumnOrThrow(columns, key);
      const int columnWidth =
          checkedSizeToInt(column.dim, "column block dimension");
      const int columnBegin =
          checkedSizeToInt(column.start, "column block offset");
      factorPlan.nonzerosPerRow =
          checkedAdd(factorPlan.nonzerosPerRow, columnWidth,
                     "factor nonzeros per row");
      factorPlan.blocks.push_back(
          {key, localBlockIndex, columnWidth, columnBegin, 0});
    }

    std::vector<size_t> sortedBlockIndices(factorPlan.blocks.size());
    std::iota(sortedBlockIndices.begin(), sortedBlockIndices.end(), 0);
    std::sort(sortedBlockIndices.begin(), sortedBlockIndices.end(),
              [&factorPlan](size_t left, size_t right) {
                return factorPlan.blocks[left].globalColumnBegin <
                       factorPlan.blocks[right].globalColumnBegin;
              });

    int valueOffset = 0;
    for (size_t blockIndex : sortedBlockIndices) {
      SparseJacobianBlockWritePlan& block = factorPlan.blocks[blockIndex];
      block.valueOffsetWithinRow = valueOffset;
      valueOffset = checkedAdd(valueOffset, block.width, "factor value offset");
      checkedAdd(block.globalColumnBegin, block.width - 1,
                 "global scalar column");
    }
    checkedMultiply(factorPlan.rowCount, factorPlan.nonzerosPerRow,
                    "factor nonzero count");
  };

#ifdef GTSAM_USE_TBB
  TbbOpenMPMixedScope threadLimiter;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, factorCount),
                    [&](const tbb::blocked_range<size_t>& range) {
                      for (size_t index = range.begin(); index != range.end();
                           ++index) {
                        buildFactorPlan(index);
                      }
                    });
#else
  for (size_t index = 0; index < factorCount; ++index) {
    buildFactorPlan(index);
  }
#endif

  // Phase B (serial): prefix sums for row offsets and nonzero counts, with
  // cumulative overflow checks matching the original serial construction.
  int totalNonzeros = 0;
  for (size_t index = 0; index < factorCount; ++index) {
    SparseJacobianFactorWritePlan& factorPlan = factors_[index];
    factorPlan.rowBegin = rows_;
    if (factorIsNull_[index]) continue;
    const int factorNonzeros = checkedMultiply(
        factorPlan.rowCount, factorPlan.nonzerosPerRow, "factor nonzero count");
    totalNonzeros =
        checkedAdd(totalNonzeros, factorNonzeros, "total nonzero count");
    rows_ = checkedAdd(rows_, factorPlan.rowCount, "total row count");
  }

  // Phase C (parallel over factors): fill row pointers and column indices;
  // each factor writes only its own row range.
  rowPointers_.assign(static_cast<size_t>(rows_) + 1, 0);
  columnIndices_.resize(static_cast<size_t>(totalNonzeros));
  // Nonzero prefix per factor, derived serially (cheap) so phase C is
  // write-disjoint.
  std::vector<size_t> nonzeroBegin(factorCount + 1, 0);
  for (size_t index = 0; index < factorCount; ++index) {
    const SparseJacobianFactorWritePlan& factorPlan = factors_[index];
    nonzeroBegin[index + 1] =
        nonzeroBegin[index] +
        (factorIsNull_[index]
             ? 0
             : static_cast<size_t>(factorPlan.rowCount) *
                   static_cast<size_t>(factorPlan.nonzerosPerRow));
  }

  const auto fillFactorCsr = [&](size_t index) {
    if (factorIsNull_[index]) return;
    const SparseJacobianFactorWritePlan& factorPlan = factors_[index];
    const size_t valueBase = nonzeroBegin[index];

    for (int localRow = 0; localRow < factorPlan.rowCount; ++localRow) {
      rowPointers_[static_cast<size_t>(factorPlan.rowBegin + localRow) + 1] =
          static_cast<int>(valueBase) +
          (localRow + 1) * factorPlan.nonzerosPerRow;
    }
    for (const SparseJacobianBlockWritePlan& block : factorPlan.blocks) {
      for (int localRow = 0; localRow < factorPlan.rowCount; ++localRow) {
        int* rowColumns =
            columnIndices_.data() + valueBase +
            static_cast<size_t>(localRow) *
                static_cast<size_t>(factorPlan.nonzerosPerRow) +
            block.valueOffsetWithinRow;
        for (int localColumn = 0; localColumn < block.width; ++localColumn) {
          rowColumns[localColumn] = block.globalColumnBegin + localColumn;
        }
      }
    }
  };

#ifdef GTSAM_USE_TBB
  tbb::parallel_for(tbb::blocked_range<size_t>(0, factorCount),
                    [&](const tbb::blocked_range<size_t>& range) {
                      for (size_t index = range.begin(); index != range.end();
                           ++index) {
                        fillFactorCsr(index);
                      }
                    });
#else
  for (size_t index = 0; index < factorCount; ++index) {
    fillFactorCsr(index);
  }
#endif

  // Coverage check (serial; a handful of milliseconds even on the largest
  // graphs). Zero-row factors intentionally provide no coverage.
  std::vector<uint8_t> coveredColumns(static_cast<size_t>(columns_), 0);
  for (size_t index = 0; index < factorCount; ++index) {
    if (factorIsNull_[index]) continue;
    const SparseJacobianFactorWritePlan& factorPlan = factors_[index];
    if (factorPlan.rowCount <= 0) continue;
    for (const SparseJacobianBlockWritePlan& block : factorPlan.blocks) {
      for (int localColumn = 0; localColumn < block.width; ++localColumn) {
        coveredColumns.at(
            static_cast<size_t>(block.globalColumnBegin + localColumn)) = 1;
      }
    }
  }
  for (const Key key : columnInfo_.ordering()) {
    const KeyInfoEntry& entry = columnInfo_.at(key);
    const int blockDimension =
        checkedSizeToInt(entry.dim, "column block dimension");
    const int blockOffset =
        checkedSizeToInt(entry.start, "column block offset");
    for (int localColumn = 0; localColumn < blockDimension; ++localColumn) {
      const int globalColumn =
          checkedAdd(blockOffset, localColumn, "covered scalar column");
      if (!coveredColumns.at(static_cast<size_t>(globalColumn))) {
        throw std::invalid_argument("SparseJacobianPlan uncovered key " +
                                    DefaultKeyFormatter(key));
      }
    }
  }

  structuralFingerprint_ =
      computeStoredFingerprint(columnInfo_, factors_, factorIsNull_);
}

int SparseJacobianPlan::rows() const { return rows_; }

int SparseJacobianPlan::columns() const { return columns_; }

int SparseJacobianPlan::nonzeros() const { return rowPointers_.back(); }

const std::vector<int>& SparseJacobianPlan::rowPointers() const {
  return rowPointers_;
}

const std::vector<int>& SparseJacobianPlan::columnIndices() const {
  return columnIndices_;
}

const SparseJacobianFactorWritePlan& SparseJacobianPlan::factor(
    size_t index) const {
  return factors_.at(index);
}

const std::vector<SparseJacobianFactorWritePlan>& SparseJacobianPlan::factors()
    const {
  return factors_;
}

uint64_t SparseJacobianPlan::structuralFingerprint() const {
  return structuralFingerprint_;
}

bool SparseJacobianPlan::matches(
    const NonlinearFactorGraph& graph, const KeyInfo& columns) const {
  try {
    if (computeGraphFingerprint(graph, columns) != structuralFingerprint_) {
      return false;
    }

    if (checkedSizeToInt(columns.numCols(), "total column count") != columns_ ||
        !sameKeyInfo(columnInfo_, columns) ||
        graph.size() != factors_.size() ||
        factorIsNull_.size() != factors_.size()) {
      return false;
    }

    int rowBegin = 0;
    for (size_t index = 0; index < graph.size(); ++index) {
      const NonlinearFactor::shared_ptr& factor = graph[index];
      const SparseJacobianFactorWritePlan& expected = factors_[index];
      const bool isNull = !factor;
      if (isNull != factorIsNull_[index] || expected.rowBegin != rowBegin) {
        return false;
      }

      if (isNull) {
        if (expected.rowCount != 0 || expected.nonzerosPerRow != 0 ||
            !expected.blocks.empty() || !expected.sendable) {
          return false;
        }
        continue;
      }

      const int rowCount = checkedSizeToInt(factor->dim(), "factor row count");
      if (rowCount != expected.rowCount ||
          factor->sendable() != expected.sendable ||
          factor->keys().size() != expected.blocks.size()) {
        return false;
      }

      for (size_t localBlockIndex = 0; localBlockIndex < factor->keys().size();
           ++localBlockIndex) {
        const Key key = factor->keys()[localBlockIndex];
        const SparseJacobianBlockWritePlan& expectedBlock =
            expected.blocks[localBlockIndex];
        const KeyInfoEntry& column = columns.at(key);
        if (expectedBlock.key != key ||
            expectedBlock.localBlockIndex != localBlockIndex ||
            expectedBlock.width !=
                checkedSizeToInt(column.dim, "column block dimension") ||
            expectedBlock.globalColumnBegin !=
                checkedSizeToInt(column.start, "column block offset")) {
          return false;
        }
      }
      rowBegin = checkedAdd(rowBegin, rowCount, "total row count");
    }
    return rowBegin == rows_;
  } catch (...) {
    return false;
  }
}

}  // namespace gtsam::cuda
