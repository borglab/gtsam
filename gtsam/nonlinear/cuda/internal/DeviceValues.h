#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/nonlinear/cuda/internal/DeviceVariableIndex.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gtsam::cuda {

template <typename T>
struct DeviceValueBlock {
  DeviceArray<T> values;
  DeviceArray<double> delta;
  std::vector<Key> keys;
  int tangentDim = 0;
};

class DeviceValues {
 public:
  const DeviceVariableIndex& index() const { return index_; }

  template <typename T>
  DeviceValueBlock<T>& addBlock(uint32_t typeId, int tangentDim,
                                const std::vector<Key>& keys,
                                const std::vector<T>& hostValues,
                                cudaStream_t stream = nullptr,
                                DeviceTransferTiming* valuesUploadTiming =
                                    nullptr,
                                double* deltaResizeElapsed = nullptr) {
    if (keys.size() != hostValues.size()) {
      throw std::invalid_argument("DeviceValues keys and values size mismatch");
    }
    if (blocks_.count(typeId) != 0) {
      throw std::invalid_argument("DeviceValues duplicate type block");
    }
    if (tangentDim <= 0) {
      throw std::invalid_argument("DeviceValues invalid tangent dimension");
    }
    if (keys.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
      throw std::invalid_argument("DeviceValues too many keys");
    }

    auto storage = std::make_unique<TypedBlock<T>>();
    storage->block.tangentDim = tangentDim;
    storage->block.keys = keys;
    if (valuesUploadTiming) {
      *valuesUploadTiming =
          storage->block.values.uploadProfiled(hostValues, stream);
    } else {
      storage->block.values.upload(hostValues, stream);
    }
    const auto deltaResizeStart =
        deltaResizeElapsed ? std::chrono::steady_clock::now()
                           : std::chrono::steady_clock::time_point{};
    storage->block.delta.resize(hostValues.size() *
                                static_cast<size_t>(tangentDim));
    if (deltaResizeElapsed) {
      *deltaResizeElapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        deltaResizeStart)
              .count();
    }

    DeviceValueBlock<T>* result = &storage->block;
    blocks_.emplace(typeId, std::move(storage));
    indexKeys(typeId, tangentDim, keys);
    return *result;
  }

  template <typename T>
  DeviceValueBlock<T>& addUninitializedBlock(uint32_t typeId, int tangentDim,
                                             const std::vector<Key>& keys) {
    if (blocks_.count(typeId) != 0) {
      throw std::invalid_argument("DeviceValues duplicate type block");
    }
    if (tangentDim <= 0) {
      throw std::invalid_argument("DeviceValues invalid tangent dimension");
    }
    if (keys.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
      throw std::invalid_argument("DeviceValues too many keys");
    }

    auto storage = std::make_unique<TypedBlock<T>>();
    storage->block.tangentDim = tangentDim;
    storage->block.keys = keys;
    storage->block.values.resize(keys.size());
    storage->block.delta.resize(keys.size() * static_cast<size_t>(tangentDim));

    DeviceValueBlock<T>* result = &storage->block;
    blocks_.emplace(typeId, std::move(storage));
    indexKeys(typeId, tangentDim, keys);
    return *result;
  }

  template <typename T>
  DeviceValueBlock<T>& block(uint32_t typeId) {
    auto* typed = dynamic_cast<TypedBlock<T>*>(blockStorage(typeId));
    if (!typed) {
      throw std::invalid_argument("DeviceValues type mismatch");
    }
    return typed->block;
  }

  template <typename T>
  const DeviceValueBlock<T>& block(uint32_t typeId) const {
    const auto* typed = dynamic_cast<const TypedBlock<T>*>(blockStorage(typeId));
    if (!typed) {
      throw std::invalid_argument("DeviceValues type mismatch");
    }
    return typed->block;
  }

 private:
  struct BlockStorage {
    virtual ~BlockStorage() = default;
  };

  template <typename T>
  struct TypedBlock final : BlockStorage {
    DeviceValueBlock<T> block;
  };

  BlockStorage* blockStorage(uint32_t typeId) {
    const auto it = blocks_.find(typeId);
    if (it == blocks_.end()) {
      throw std::out_of_range("DeviceValues missing type block");
    }
    return it->second.get();
  }

  const BlockStorage* blockStorage(uint32_t typeId) const {
    const auto it = blocks_.find(typeId);
    if (it == blocks_.end()) {
      throw std::out_of_range("DeviceValues missing type block");
    }
    return it->second.get();
  }

  /**
   * Registers a freshly added block's keys, mapping each to its slot.
   *
   * Duplicates are detected against the index as it is filled, which covers
   * both keys already held by another block and repeats within `keys`. That
   * makes a separate pre-pass over a temporary key set unnecessary: for a
   * bundle-adjustment problem with one key per landmark, the pre-pass built and
   * threw away a hash table the same size as the index itself. On failure the
   * keys added so far and the block are removed, leaving the object unchanged.
   */
  void indexKeys(uint32_t typeId, int tangentDim,
                 const std::vector<Key>& keys) {
    index_.reserve(index_.size() + keys.size());
    size_t committed = 0;
    try {
      for (size_t i = 0; i < keys.size(); ++i) {
        if (index_.contains(keys[i])) {
          throw std::invalid_argument("DeviceValues duplicate key");
        }
        index_.add(keys[i], typeId, static_cast<int>(i), tangentDim);
        ++committed;
      }
    } catch (...) {
      for (size_t i = 0; i < committed; ++i) {
        index_.erase(keys[i]);
      }
      blocks_.erase(typeId);
      throw;
    }
  }

  DeviceVariableIndex index_;
  std::unordered_map<uint32_t, std::unique_ptr<BlockStorage>> blocks_;
};

}  // namespace gtsam::cuda
