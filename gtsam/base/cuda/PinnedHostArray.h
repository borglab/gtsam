/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    PinnedHostArray.h
 * @brief   Owning pinned (page-locked) host allocation
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#pragma once

#include <cuda_runtime_api.h>
#include <gtsam/base/cuda/Errors.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace gtsam::cuda {

template <typename T>
class PinnedHostArray {
  static_assert(std::is_trivially_copyable_v<T>,
                "PinnedHostArray requires trivially copyable values");

 public:
  PinnedHostArray() = default;

  explicit PinnedHostArray(size_t size) { resize(size); }

  PinnedHostArray(const PinnedHostArray&) = delete;
  PinnedHostArray& operator=(const PinnedHostArray&) = delete;

  PinnedHostArray(PinnedHostArray&& other) noexcept
      : data_(other.data_), size_(other.size_) {
    other.data_ = nullptr;
    other.size_ = 0;
  }

  PinnedHostArray& operator=(PinnedHostArray&& other) noexcept {
    if (this == &other) return *this;
    resetUnchecked();
    data_ = other.data_;
    size_ = other.size_;
    other.data_ = nullptr;
    other.size_ = 0;
    return *this;
  }

  ~PinnedHostArray() { resetUnchecked(); }

  void resize(size_t size) {
    if (size == size_) return;
    if (size > std::numeric_limits<size_t>::max() / sizeof(T)) {
      throw std::overflow_error("PinnedHostArray allocation size overflow");
    }

    PinnedHostArray replacement;
    if (size > 0) {
      GTSAM_CUDA_CHECK(cudaMallocHost(
          reinterpret_cast<void**>(&replacement.data_), sizeof(T) * size));
      replacement.size_ = size;
    }

    if (data_) {
      GTSAM_CUDA_CHECK(cudaFreeHost(data_));
    }
    data_ = replacement.data_;
    size_ = replacement.size_;
    replacement.data_ = nullptr;
    replacement.size_ = 0;
  }

  // Value-initializes every element in the allocation.
  void clear() {
    if (size_ == 0) return;
    std::fill(data_, data_ + size_, T{});
  }

  T* data() { return data_; }
  const T* data() const { return data_; }
  size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }

 private:
  T* data_ = nullptr;
  size_t size_ = 0;

  void resetUnchecked() noexcept {
    if (data_) {
      cudaFreeHost(data_);
    }
    data_ = nullptr;
    size_ = 0;
  }
};

}  // namespace gtsam::cuda
