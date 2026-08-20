#include <gtsam/linear/cuda/internal/BlockOrdering.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace gtsam::cuda {
namespace {

std::unordered_map<Key, VariableBlock> buildBlockMap(
    const BlockLayout& blocks) {
  std::unordered_map<Key, VariableBlock> byKey;
  byKey.reserve(blocks.size());
  for (const VariableBlock& block : blocks) {
    if (!byKey.emplace(block.key, block).second) {
      throw std::invalid_argument("CUDA block layout contains a duplicate key");
    }
  }
  return byKey;
}

}  // namespace

int validateBlockLayout(const BlockLayout& blocks) {
  std::vector<VariableBlock> ordered = blocks;
  std::sort(ordered.begin(), ordered.end(),
            [](const VariableBlock& left,
               const VariableBlock& right) {
              return left.scalarOffset < right.scalarOffset;
            });

  int nextOffset = 0;
  for (const VariableBlock& block : ordered) {
    if (block.dimension <= 0) {
      throw std::invalid_argument(
          "CUDA block layout dimensions must be positive");
    }
    if (block.scalarOffset != nextOffset) {
      throw std::invalid_argument(
          "CUDA block layout must cover a contiguous scalar interval");
    }
    if (block.dimension > std::numeric_limits<int>::max() - nextOffset) {
      throw std::invalid_argument("CUDA block layout dimension overflows int");
    }
    nextOffset += block.dimension;
  }
  buildBlockMap(blocks);
  return nextOffset;
}

std::vector<int> compileScalarPermutation(
    const BlockLayout& blocks, const Ordering& ordering) {
  const int scalarDimension = validateBlockLayout(blocks);
  if (ordering.size() != blocks.size()) {
    throw std::invalid_argument(
        "CUDA ordering must contain every block key exactly once");
  }

  const auto byKey = buildBlockMap(blocks);
  std::unordered_set<Key> consumed;
  consumed.reserve(ordering.size());
  std::vector<int> permutation;
  permutation.reserve(static_cast<size_t>(scalarDimension));

  for (const Key key : ordering) {
    const auto found = byKey.find(key);
    if (found == byKey.end()) {
      throw std::invalid_argument("CUDA ordering contains an unknown key");
    }
    if (!consumed.insert(key).second) {
      throw std::invalid_argument("CUDA ordering contains a duplicate key");
    }
    const VariableBlock& block = found->second;
    for (int scalar = 0; scalar < block.dimension; ++scalar) {
      permutation.push_back(block.scalarOffset + scalar);
    }
  }

  if (permutation.size() != static_cast<size_t>(scalarDimension)) {
    throw std::invalid_argument(
        "CUDA ordering does not cover the scalar dimension");
  }
  return permutation;
}

}  // namespace gtsam::cuda
