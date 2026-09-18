/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DeviceValues.h
 * @brief   Device-resident Values storage with tangent-space retraction
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

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

/**
 * All variables of one device type, packed into two flat device arrays.
 *
 * Values and their tangent-space increments are stored separately and in
 * parallel: `values[i]` is retracted by the tangentDim scalars of `delta`
 * starting at `i * tangentDim` to give the next iterate, which is what lets a
 * retraction kernel run one thread per variable over whole arrays. The key list
 * stays on the host and gives the Key of slot i, for reading results back into a
 * gtsam::Values.
 */
template <typename T>
struct DeviceValueBlock {
  /// Current value of each variable, one entry per slot.
  DeviceArray<T> values;
  /// Tangent increments, tangentDim consecutive scalars per slot.
  DeviceArray<double> delta;
  /// Host-side slot-to-Key map, parallel to `values`.
  std::vector<Key> keys;
  /// Tangent-space width of this type.
  int tangentDim = 0;
};

/**
 * Device-resident counterpart of gtsam::Values: the variables of an
 * optimization problem, packed by type into DeviceValueBlocks.
 *
 * A gtsam::Values is a map from Key to a type-erased value, which a kernel can
 * neither traverse nor dereference. Here each supported manifold type gets one
 * block of contiguous device memory and variables are addressed by dense integer
 * slot, with DeviceVariableIndex holding the Key-to-slot map that host code uses
 * to translate. Blocks are added once, up front, and then reused across every
 * iteration: the device arrays keep their addresses, so retraction updates them
 * in place rather than reallocating.
 *
 * Type erasure is by an internal polymorphic holder, so one object can carry
 * blocks of unrelated types while each block stays statically typed; block<T>()
 * checks the requested type against what was stored.
 */
class DeviceValues {
 public:
  /// The Key-to-slot map covering every added block.
  const DeviceVariableIndex& index() const { return index_; }

  /**
   * Adds a block of type T, uploading `hostValues` and sizing its delta array.
   *
   * Throws std::invalid_argument if the key and value counts disagree, the type
   * already has a block, the tangent dimension is not positive, there are more
   * keys than an int can index, or a key is already registered. The optional out
   * parameters report upload and allocation cost for the timing breakdown; pass
   * nullptr to skip measuring them.
   */
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

  /// Adds a block of type T with its arrays allocated but not written, for when
  /// a kernel produces the initial values instead of the host uploading them.
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

  /// Returns the block registered under `typeId`, throwing std::out_of_range if
  /// there is none and std::invalid_argument if it does not hold type T.
  template <typename T>
  DeviceValueBlock<T>& block(uint32_t typeId) {
    auto* typed = dynamic_cast<TypedBlock<T>*>(blockStorage(typeId));
    if (!typed) {
      throw std::invalid_argument("DeviceValues type mismatch");
    }
    return typed->block;
  }

  /// Const overload of block().
  template <typename T>
  const DeviceValueBlock<T>& block(uint32_t typeId) const {
    const auto* typed = dynamic_cast<const TypedBlock<T>*>(blockStorage(typeId));
    if (!typed) {
      throw std::invalid_argument("DeviceValues type mismatch");
    }
    return typed->block;
  }

 private:
  /// Type-erased handle to a block, so blocks of unrelated types can share one
  /// map. Virtual only to make the held type recoverable by dynamic_cast.
  struct BlockStorage {
    virtual ~BlockStorage() = default;
  };

  /// The one derived type, holding a statically typed block.
  template <typename T>
  struct TypedBlock final : BlockStorage {
    DeviceValueBlock<T> block;
  };

  /// Looks up a block by tag, throwing std::out_of_range if absent.
  BlockStorage* blockStorage(uint32_t typeId) {
    const auto it = blocks_.find(typeId);
    if (it == blocks_.end()) {
      throw std::out_of_range("DeviceValues missing type block");
    }
    return it->second.get();
  }

  /// Const overload of blockStorage().
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

  /// Key-to-slot map, kept in step with the blocks below.
  DeviceVariableIndex index_;
  /// One block per registered device type, keyed by its type tag.
  std::unordered_map<uint32_t, std::unique_ptr<BlockStorage>> blocks_;
};

}  // namespace gtsam::cuda
