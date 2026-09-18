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

/**
 * Owning host buffer in page-locked memory, the staging area for transfers.
 *
 * Where DeviceArray owns memory on the GPU, this owns memory on the host, so
 * the two are the endpoints of a copy rather than alternatives. What makes it
 * worth a separate class from std::vector is that cudaMallocHost pins the pages
 * so the operating system cannot move or swap them, which buys two things the
 * driver cannot offer for ordinary host memory:
 *
 *  - the DMA engine can read and write the buffer directly, roughly doubling
 *    achieved bandwidth instead of staging through an internal pinned buffer;
 *    and
 *  - cudaMemcpyAsync is genuinely asynchronous, so a copy can overlap with
 *    compute rather than blocking until it completes.
 *
 * The cost is that pinning is a scarce, slow-to-acquire resource: allocate
 * these once and reuse them across iterations rather than per solve. Because
 * asynchronous copies read the buffer after the call returns, it must stay
 * alive and unmodified until the stream reaches that copy. Move-only, since two
 * owners would double-free.
 */
template <typename T>
class PinnedHostArray {
  static_assert(std::is_trivially_copyable_v<T>,
                "PinnedHostArray requires trivially copyable values");

 public:
  /// Constructs an empty array that allocates nothing.
  PinnedHostArray() = default;

  /// Allocates room for size elements, leaving their values uninitialized.
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

  /**
   * Reallocates to hold exactly size elements, discarding any current contents.
   *
   * A request for the size already held is ignored, so this is cheap to call
   * every iteration to keep a reused buffer at the right size. Any other size
   * allocates the replacement before freeing the old buffer, so a failed
   * allocation throws and leaves the existing one intact. Values in the new
   * buffer are uninitialized; call clear() if they need to be zero. Must not be
   * called while an asynchronous copy on this buffer is still in flight.
   */
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

  /// Value-initializes every element in the allocation.
  void clear() {
    if (size_ == 0) return;
    std::fill(data_, data_ + size_, T{});
  }

  /// Returns a pointer to the pinned storage, null when empty.
  T* data() { return data_; }
  /// Returns a pointer to the pinned storage, null when empty.
  const T* data() const { return data_; }
  /// Returns the number of elements, not bytes.
  size_t size() const { return size_; }
  /// Returns whether nothing is allocated.
  bool empty() const { return size_ == 0; }

 private:
  T* data_ = nullptr;
  size_t size_ = 0;

  /// Frees without checking, for use in the destructor and move assignment.
  void resetUnchecked() noexcept {
    if (data_) {
      cudaFreeHost(data_);
    }
    data_ = nullptr;
    size_ = 0;
  }
};

}  // namespace gtsam::cuda
