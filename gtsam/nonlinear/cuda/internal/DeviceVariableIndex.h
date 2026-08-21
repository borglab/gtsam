/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DeviceVariableIndex.h
 * @brief   Device variable-to-column index for the sparse Jacobian pipeline
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

#pragma once

#include <gtsam/base/types.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>

namespace gtsam::cuda {

/**
 * Where one variable lives in DeviceValues: which typed block, and which
 * position within it.
 *
 * The type tag is carried alongside the slot so a lookup can be checked against
 * the type the caller expects, which is the only guard available once the values
 * are flat device arrays with no runtime type of their own.
 */
struct DeviceVariableSlot {
  /// Variable this slot describes.
  Key key = 0;
  /// Tag of the DeviceValues block holding it, e.g. kDevicePoint3Type.
  uint32_t typeId = 0;
  /// Position within that block's packed array.
  int slot = -1;
  /// Tangent-space width of the variable.
  int tangentDim = 0;
};

/**
 * Key-to-slot map for the variables uploaded to a DeviceValues.
 *
 * This is the device pipeline's counterpart to looking a Key up in a
 * gtsam::Values: kernels address variables by dense integer slot, so every
 * host-side step that starts from a Key -- building observations, assembling the
 * sparse Jacobian, reading results back -- passes through here first. Owned by
 * DeviceValues, which adds entries as blocks are added and rolls them back if a
 * block fails to register.
 */
class DeviceVariableIndex {
 public:
  /// Registers one variable, throwing if the key is already present or the slot
  /// or tangent dimension is not positive.
  void add(Key key, uint32_t typeId, int slot, int tangentDim) {
    if (entries_.count(key) != 0) {
      throw std::invalid_argument("DeviceVariableIndex duplicate key");
    }
    if (slot < 0) {
      throw std::invalid_argument("DeviceVariableIndex invalid slot");
    }
    if (tangentDim <= 0) {
      throw std::invalid_argument("DeviceVariableIndex invalid tangent dim");
    }
    entries_.emplace(key, DeviceVariableSlot{key, typeId, slot, tangentDim});
  }

  /// Returns the slot for `key`, throwing std::out_of_range if absent.
  const DeviceVariableSlot& at(Key key) const {
    const auto it = entries_.find(key);
    if (it == entries_.end()) {
      throw std::out_of_range("DeviceVariableIndex missing key");
    }
    return it->second;
  }

  /// Whether `key` has been registered.
  bool contains(Key key) const { return entries_.count(key) != 0; }

  /// Returns just the slot, throwing unless the entry's type tag matches, which
  /// catches a Key being read as the wrong device type.
  int slot(Key key, uint32_t expectedTypeId) const {
    const DeviceVariableSlot& entry = at(key);
    if (entry.typeId != expectedTypeId) {
      throw std::invalid_argument("DeviceVariableIndex type mismatch");
    }
    return entry.slot;
  }

  /// Number of registered variables.
  size_t size() const { return entries_.size(); }
  /// Whether no variables are registered.
  bool empty() const { return entries_.empty(); }

  /// Removes `key` if present, used to unwind a partially added block.
  void erase(Key key) { entries_.erase(key); }

  /// Pre-sizes the table for `count` total entries, avoiding growth rehashes.
  void reserve(size_t count) { entries_.reserve(count); }

 private:
  std::unordered_map<Key, DeviceVariableSlot> entries_;
};

}  // namespace gtsam::cuda
