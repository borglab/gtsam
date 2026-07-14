#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>

#include <algorithm>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace gtsam::cuda {
namespace {

constexpr uint64_t kFnv1aOffsetBasis = 14695981039346656037ULL;
constexpr uint64_t kFnv1aPrime = 1099511628211ULL;

int CheckedSizeToInt(size_t value, const char* description) {
  if (value > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(std::string(description) +
                                " exceeds signed int capacity");
  }
  return static_cast<int>(value);
}

int CheckedAdd(int left, int right, const char* description) {
  if (left < 0 || right < 0 || right > std::numeric_limits<int>::max() - left) {
    throw std::invalid_argument(std::string(description) +
                                " exceeds signed int capacity");
  }
  return left + right;
}

int CheckedMultiply(int left, int right, const char* description) {
  if (left < 0 || right < 0 ||
      (left != 0 && right > std::numeric_limits<int>::max() / left)) {
    throw std::invalid_argument(std::string(description) +
                                " exceeds signed int capacity");
  }
  return left * right;
}

const SparseJacobianColumnBlock& FindColumnOrThrow(
    const SparseJacobianColumnLayout& columns, Key key) {
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

void AppendColumns(Fnv1a64* hash,
                   const std::vector<SparseJacobianColumnBlock>& blocks) {
  hash->appendSize(blocks.size());
  for (const SparseJacobianColumnBlock& block : blocks) {
    hash->appendUint64(static_cast<uint64_t>(block.key));
    hash->appendInt(block.dimension);
    hash->appendInt(block.columnBegin);
  }
}

uint64_t ComputeStoredFingerprint(
    const std::vector<SparseJacobianColumnBlock>& columnBlocks,
    const std::vector<SparseJacobianFactorWritePlan>& factors,
    const std::vector<bool>& factorIsNull) {
  Fnv1a64 hash;
  AppendColumns(&hash, columnBlocks);
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

uint64_t ComputeGraphFingerprint(const NonlinearFactorGraph& graph,
                                 const SparseJacobianColumnLayout& columns) {
  Fnv1a64 hash;
  AppendColumns(&hash, columns.blocks());
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

    hash.appendInt(CheckedSizeToInt(factor->dim(), "factor row count"));
    hash.appendSize(factor->keys().size());
    for (Key key : factor->keys()) {
      const SparseJacobianColumnBlock& column = FindColumnOrThrow(columns, key);
      hash.appendUint64(static_cast<uint64_t>(key));
      hash.appendInt(column.dimension);
    }
    hash.appendBool(factor->sendable());
  }
  return hash.value();
}

bool SameColumnBlocks(const std::vector<SparseJacobianColumnBlock>& expected,
                      const std::vector<SparseJacobianColumnBlock>& actual) {
  if (expected.size() != actual.size()) {
    return false;
  }
  for (size_t index = 0; index < expected.size(); ++index) {
    if (expected[index].key != actual[index].key ||
        expected[index].dimension != actual[index].dimension ||
        expected[index].columnBegin != actual[index].columnBegin) {
      return false;
    }
  }
  return true;
}

}  // namespace

SparseJacobianColumnLayout::SparseJacobianColumnLayout(const Values& values) {
  for (const auto& [key, dimensionSize] : values.dims()) {
    if (dimensionSize == 0) {
      throw std::invalid_argument(
          "SparseJacobianColumnLayout rejects zero-dimensional key " +
          DefaultKeyFormatter(key));
    }
    const int dimension =
        CheckedSizeToInt(dimensionSize, "column block dimension");
    const int nextTotal =
        CheckedAdd(totalColumns_, dimension, "total column count");

    const size_t blockIndex = blocks_.size();
    const auto insertion = keyToBlock_.emplace(key, blockIndex);
    if (!insertion.second) {
      throw std::invalid_argument("SparseJacobianColumnLayout duplicate key " +
                                  DefaultKeyFormatter(key));
    }

    blocks_.push_back({key, dimension, totalColumns_});
    totalColumns_ = nextTotal;
  }
}

const SparseJacobianColumnBlock& SparseJacobianColumnLayout::at(Key key) const {
  const auto found = keyToBlock_.find(key);
  if (found == keyToBlock_.end()) {
    throw std::out_of_range("SparseJacobianColumnLayout missing key " +
                            DefaultKeyFormatter(key));
  }
  return blocks_.at(found->second);
}

const std::vector<SparseJacobianColumnBlock>&
SparseJacobianColumnLayout::blocks() const {
  return blocks_;
}

int SparseJacobianColumnLayout::totalColumns() const { return totalColumns_; }

bool SparseJacobianColumnLayout::matches(const Values& values) const {
  const auto dimensions = values.dims();
  if (dimensions.size() != blocks_.size()) {
    return false;
  }

  try {
    int columnBegin = 0;
    size_t index = 0;
    for (const auto& [key, dimensionSize] : dimensions) {
      if (dimensionSize == 0) {
        return false;
      }
      const int dimension =
          CheckedSizeToInt(dimensionSize, "column block dimension");
      const SparseJacobianColumnBlock& block = blocks_[index++];
      if (block.key != key || block.dimension != dimension ||
          block.columnBegin != columnBegin) {
        return false;
      }
      columnBegin = CheckedAdd(columnBegin, dimension, "total column count");
    }
    return columnBegin == totalColumns_;
  } catch (const std::invalid_argument&) {
    return false;
  }
}

VectorValues SparseJacobianColumnLayout::toVectorValues(
    const Vector& flatDelta) const {
  if (flatDelta.size() != totalColumns_) {
    throw std::invalid_argument(
        "SparseJacobianColumnLayout delta dimension mismatch");
  }

  VectorValues result;
  for (const SparseJacobianColumnBlock& block : blocks_) {
    result.insert(block.key, Vector(flatDelta.segment(block.columnBegin,
                                                      block.dimension)));
  }
  return result;
}

SparseJacobianPlan::SparseJacobianPlan(
    const NonlinearFactorGraph& graph,
    const SparseJacobianColumnLayout& columns)
    : columns_(columns.totalColumns()), columnBlocks_(columns.blocks()) {
  std::vector<bool> coveredColumns(static_cast<size_t>(columns_), false);
  factors_.reserve(graph.size());
  factorIsNull_.reserve(graph.size());

  for (const NonlinearFactor::shared_ptr& factor : graph) {
    SparseJacobianFactorWritePlan factorPlan;
    factorPlan.rowBegin = rows_;

    const bool isNull = !factor;
    factorIsNull_.push_back(isNull);
    if (isNull) {
      factors_.push_back(std::move(factorPlan));
      continue;
    }

    factorPlan.rowCount = CheckedSizeToInt(factor->dim(), "factor row count");
    factorPlan.sendable = factor->sendable();
    const int nextRow =
        CheckedAdd(rows_, factorPlan.rowCount, "total row count");

    std::set<Key> seenKeys;
    factorPlan.blocks.reserve(factor->keys().size());
    for (size_t localBlockIndex = 0; localBlockIndex < factor->keys().size();
         ++localBlockIndex) {
      const Key key = factor->keys()[localBlockIndex];
      if (!seenKeys.insert(key).second) {
        throw std::invalid_argument("SparseJacobianPlan repeated factor key " +
                                    DefaultKeyFormatter(key));
      }

      const SparseJacobianColumnBlock& column = FindColumnOrThrow(columns, key);
      factorPlan.nonzerosPerRow =
          CheckedAdd(factorPlan.nonzerosPerRow, column.dimension,
                     "factor nonzeros per row");
      factorPlan.blocks.push_back(
          {key, localBlockIndex, column.dimension, column.columnBegin, 0});
    }

    std::vector<size_t> sortedBlockIndices(factorPlan.blocks.size());
    std::iota(sortedBlockIndices.begin(), sortedBlockIndices.end(), 0);
    std::sort(sortedBlockIndices.begin(), sortedBlockIndices.end(),
              [&factorPlan](size_t left, size_t right) {
                return factorPlan.blocks[left].globalColumnBegin <
                       factorPlan.blocks[right].globalColumnBegin;
              });

    std::vector<int> scalarColumns;
    scalarColumns.reserve(static_cast<size_t>(factorPlan.nonzerosPerRow));
    int valueOffset = 0;
    for (size_t blockIndex : sortedBlockIndices) {
      SparseJacobianBlockWritePlan& block = factorPlan.blocks[blockIndex];
      block.valueOffsetWithinRow = valueOffset;
      valueOffset = CheckedAdd(valueOffset, block.width, "factor value offset");
      for (int localColumn = 0; localColumn < block.width; ++localColumn) {
        scalarColumns.push_back(CheckedAdd(block.globalColumnBegin, localColumn,
                                           "global scalar column"));
      }
    }

    const int factorNonzeros = CheckedMultiply(
        factorPlan.rowCount, factorPlan.nonzerosPerRow, "factor nonzero count");
    CheckedAdd(rowPointers_.back(), factorNonzeros, "total nonzero count");
    for (int localRow = 0; localRow < factorPlan.rowCount; ++localRow) {
      columnIndices_.insert(columnIndices_.end(), scalarColumns.begin(),
                            scalarColumns.end());
      rowPointers_.push_back(CheckedAdd(rowPointers_.back(),
                                        factorPlan.nonzerosPerRow,
                                        "total nonzero count"));
    }

    if (factorPlan.rowCount > 0) {
      for (const SparseJacobianBlockWritePlan& block : factorPlan.blocks) {
        for (int localColumn = 0; localColumn < block.width; ++localColumn) {
          const int globalColumn = CheckedAdd(
              block.globalColumnBegin, localColumn, "covered scalar column");
          coveredColumns.at(static_cast<size_t>(globalColumn)) = true;
        }
      }
    }

    factors_.push_back(std::move(factorPlan));
    rows_ = nextRow;
  }

  for (const SparseJacobianColumnBlock& block : columnBlocks_) {
    for (int localColumn = 0; localColumn < block.dimension; ++localColumn) {
      const int globalColumn =
          CheckedAdd(block.columnBegin, localColumn, "covered scalar column");
      if (!coveredColumns.at(static_cast<size_t>(globalColumn))) {
        throw std::invalid_argument("SparseJacobianPlan uncovered key " +
                                    DefaultKeyFormatter(block.key));
      }
    }
  }

  structuralFingerprint_ =
      ComputeStoredFingerprint(columnBlocks_, factors_, factorIsNull_);
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
    const NonlinearFactorGraph& graph,
    const SparseJacobianColumnLayout& columns) const {
  try {
    if (ComputeGraphFingerprint(graph, columns) != structuralFingerprint_) {
      return false;
    }

    if (columns.totalColumns() != columns_ ||
        !SameColumnBlocks(columnBlocks_, columns.blocks()) ||
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

      const int rowCount = CheckedSizeToInt(factor->dim(), "factor row count");
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
        const SparseJacobianColumnBlock& column = columns.at(key);
        if (expectedBlock.key != key ||
            expectedBlock.localBlockIndex != localBlockIndex ||
            expectedBlock.width != column.dimension ||
            expectedBlock.globalColumnBegin != column.columnBegin) {
          return false;
        }
      }
      rowBegin = CheckedAdd(rowBegin, rowCount, "total row count");
    }
    return rowBegin == rows_;
  } catch (...) {
    return false;
  }
}

}  // namespace gtsam::cuda
