#pragma once

#include <gtsam/base/cuda/Errors.h>

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

class Context {
 public:
  Context() { GTSAM_CUDA_CHECK(cudaStreamCreate(&stream_)); }

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

  cudaStream_t stream() const { return stream_; }

  void synchronize() const { GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream_)); }

 private:
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
