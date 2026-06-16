#pragma once

#include <gtsam/base/types.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>

namespace gtsam::cuda {

struct DeviceVariableSlot {
  Key key = 0;
  uint32_t typeId = 0;
  int slot = -1;
  int tangentDim = 0;
};

class DeviceVariableIndex {
 public:
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

  const DeviceVariableSlot& at(Key key) const {
    const auto it = entries_.find(key);
    if (it == entries_.end()) {
      throw std::out_of_range("DeviceVariableIndex missing key");
    }
    return it->second;
  }

  bool contains(Key key) const { return entries_.count(key) != 0; }

  int slot(Key key, uint32_t expectedTypeId) const {
    const DeviceVariableSlot& entry = at(key);
    if (entry.typeId != expectedTypeId) {
      throw std::invalid_argument("DeviceVariableIndex type mismatch");
    }
    return entry.slot;
  }

  size_t size() const { return entries_.size(); }
  bool empty() const { return entries_.empty(); }

  void erase(Key key) { entries_.erase(key); }

 private:
  std::unordered_map<Key, DeviceVariableSlot> entries_;
};

}  // namespace gtsam::cuda
