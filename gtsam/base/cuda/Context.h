/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Context.h
 * @brief   RAII owner of the CUDA stream and library handles
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

#pragma once

#include <gtsam/base/cuda/Errors.h>

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

/**
 * Owns the CUDA stream that every device operation in a solver is ordered on.
 *
 * All GTSAM CUDA work is issued on one stream, so kernels, copies, and library
 * calls run in program order and only explicit synchronization is needed. The
 * stream is either created and destroyed here, or borrowed from a caller that
 * already has one; a borrowed stream outlives this object and is never
 * destroyed by it. Move-only, because two owners would double-destroy.
 */
class Context {
 public:
  /// Creates and owns a new stream.
  Context() { GTSAM_CUDA_CHECK(cudaStreamCreate(&stream_)); }

  /// Borrows a stream owned elsewhere, which must outlive this object.
  explicit Context(cudaStream_t externalStream)
      : stream_(externalStream), ownsStream_(false) {}

  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;

  Context(Context&& other) noexcept
      : stream_(other.stream_), ownsStream_(other.ownsStream_) {
    other.stream_ = nullptr;
    other.ownsStream_ = false;
  }

  Context& operator=(Context&& other) noexcept {
    if (this == &other) return *this;
    reset();
    stream_ = other.stream_;
    ownsStream_ = other.ownsStream_;
    other.stream_ = nullptr;
    other.ownsStream_ = false;
    return *this;
  }

  ~Context() { reset(); }

  /// Returns the stream to issue work on, null after a move.
  cudaStream_t stream() const { return stream_; }

  /// Blocks the calling thread until all work issued so far has completed.
  void synchronize() const { GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream_)); }

 private:
  /// Destroys the stream if owned, and leaves the object empty either way.
  void reset() {
    if (ownsStream_ && stream_) {
      cudaStreamDestroy(stream_);
    }
    stream_ = nullptr;
    ownsStream_ = false;
  }

  cudaStream_t stream_ = nullptr;
  bool ownsStream_ = true;
};

}  // namespace gtsam::cuda
