#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceVariableIndex.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace gtsam::cuda {

template <typename T>
struct DeviceValueBlock {
  CudaDeviceArray<T> values;
  CudaDeviceArray<double> delta;
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
                                cudaStream_t stream = nullptr) {
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

    std::unordered_set<Key> newKeys;
    newKeys.reserve(keys.size());
    for (Key key : keys) {
      if (index_.contains(key)) {
        throw std::invalid_argument("DeviceValues duplicate key");
      }
      if (!newKeys.insert(key).second) {
        throw std::invalid_argument("DeviceValues duplicate key");
      }
    }

    auto storage = std::make_unique<TypedBlock<T>>();
    storage->block.tangentDim = tangentDim;
    storage->block.keys = keys;
    storage->block.values.upload(hostValues, stream);
    storage->block.delta.resize(hostValues.size() *
                                static_cast<size_t>(tangentDim));

    DeviceValueBlock<T>* result = &storage->block;
    std::vector<Key> committedKeys;
    committedKeys.reserve(keys.size());
    blocks_.emplace(typeId, std::move(storage));

    try {
      for (size_t i = 0; i < keys.size(); ++i) {
        committedKeys.push_back(keys[i]);
        index_.add(keys[i], typeId, static_cast<int>(i), tangentDim);
      }
    } catch (...) {
      for (Key key : committedKeys) {
        index_.erase(key);
      }
      blocks_.erase(typeId);
      throw;
    }

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

    std::unordered_set<Key> newKeys;
    newKeys.reserve(keys.size());
    for (Key key : keys) {
      if (index_.contains(key)) {
        throw std::invalid_argument("DeviceValues duplicate key");
      }
      if (!newKeys.insert(key).second) {
        throw std::invalid_argument("DeviceValues duplicate key");
      }
    }

    auto storage = std::make_unique<TypedBlock<T>>();
    storage->block.tangentDim = tangentDim;
    storage->block.keys = keys;
    storage->block.values.resize(keys.size());
    storage->block.delta.resize(keys.size() * static_cast<size_t>(tangentDim));

    DeviceValueBlock<T>* result = &storage->block;
    std::vector<Key> committedKeys;
    committedKeys.reserve(keys.size());
    blocks_.emplace(typeId, std::move(storage));

    try {
      for (size_t i = 0; i < keys.size(); ++i) {
        committedKeys.push_back(keys[i]);
        index_.add(keys[i], typeId, static_cast<int>(i), tangentDim);
      }
    } catch (...) {
      for (Key key : committedKeys) {
        index_.erase(key);
      }
      blocks_.erase(typeId);
      throw;
    }

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

  DeviceVariableIndex index_;
  std::unordered_map<uint32_t, std::unique_ptr<BlockStorage>> blocks_;
};

}  // namespace gtsam::cuda
