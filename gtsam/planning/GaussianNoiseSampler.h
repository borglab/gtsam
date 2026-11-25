/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianNoiseSampler.h
 * @brief   Samples zero-mean Gaussian noise for many control sequences at
 *          once, reproducibly for any number of threads
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/config.h>  // for GTSAM_USE_TBB

#include <Eigen/Cholesky>

#ifdef GTSAM_USE_TBB
#include <tbb/parallel_for.h>
#endif

#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>

namespace gtsam {

/**
 * Samples zero-mean Gaussian noise N(0, covariance) for K control sequences
 * over a horizon of H steps, as MPPI needs once per step.
 *
 * The noise is E = L Z, with covariance = L L^T and Z standard normal, in an
 * M x (K*H) matrix whose column i*H + k holds sample i at step k, where M is
 * the dimension of the covariance. Sample i draws from its own SplitMix64
 * generator, seeded from (seed, stepCounter(), i), so the noise is the same
 * for any number of threads; with TBB the samples are drawn in parallel.
 *
 * Sampling writes into reused member buffers, so one sampler must not sample
 * from several threads at once.
 */
class GaussianNoiseSampler {
 public:
  /**
   * Construct for the given noise covariance, number of samples K and seed.
   * The covariance may be left empty, but sampling then throws.
   * @throws std::invalid_argument if the covariance is set but is not a
   * square positive-definite matrix.
   */
  GaussianNoiseSampler(const Matrix& covariance, size_t numSamples,
                       std::uint64_t seed)
      : cholesky_(lowerCholesky(covariance)),
        numSamples_(numSamples),
        seed_(seed) {}

  /// The noise dimension M, or 0 if no covariance is set.
  size_t dim() const { return static_cast<size_t>(cholesky_.rows()); }

  /// The number of samples K.
  size_t numSamples() const { return numSamples_; }

  /// The seed of the per-sample generators.
  std::uint64_t seed() const { return seed_; }

  /// The number of sampling calls so far; it keys the per-sample generators.
  std::uint64_t stepCounter() const { return stepCounter_; }

  /**
   * Sample E = L Z for the given horizon H, in an M x (K*H) matrix whose
   * column i*H + k holds sample i at step k. Each call advances
   * stepCounter().
   * @return E, in a reused buffer that stays valid until the next sampling
   * call.
   * @throws std::invalid_argument if no covariance is set.
   */
  const Matrix& sampleNoise(size_t horizon) const {
    drawNoise(horizon, [](Eigen::Index, size_t) {});
    scaledNoise_.noalias() = cholesky_ * noise_;
    return scaledNoise_;
  }

  /**
   * Sample K control sequences around the nominal one, v = u + eps, with eps
   * the noise that sampleNoise() would return for this call. nominalControls
   * has one column per step of the horizon (M x H). The result uses the same
   * layout as sampleNoise(): column i*H + k holds sample i at step k. The
   * result equals sampleNoise() plus the nominal controls exactly. Each call
   * advances stepCounter().
   * @return the sampled controls, in a reused buffer that stays valid until
   * the next sampling call.
   * @throws std::invalid_argument if no covariance is set, or its dimension
   * differs from the rows of nominalControls.
   */
  const Matrix& sampleControls(const Matrix& nominalControls) const {
    if (cholesky_.size() != 0 && cholesky_.rows() != nominalControls.rows()) {
      throw std::invalid_argument(
          "GaussianNoiseSampler::sampleControls: the rows of nominalControls "
          "must match the dimension of the covariance.");
    }
    const size_t horizon = nominalControls.cols();
    sampledControls_.resize(nominalControls.rows(), numSamples_ * horizon);
    // v = (L z) + u for each column, with L lower triangular.
    drawNoise(horizon, [&](Eigen::Index column, size_t k) {
      for (Eigen::Index row = 0; row < cholesky_.rows(); ++row) {
        double scaled = 0.0;
        for (Eigen::Index j = 0; j <= row; ++j) {
          scaled += cholesky_(row, j) * noise_(j, column);
        }
        sampledControls_(row, column) = scaled + nominalControls(row, k);
      }
    });
    return sampledControls_;
  }

 private:
  /**
   * Draw the standard normal noise Z for the given horizon into noise_ and
   * advance the step counter. Sample i draws from its own generator, in
   * parallel with TBB. Right after drawing step k of sample i, the thread
   * calls perColumn(column, k) with that column of Z, column = i*H + k.
   * @throws std::invalid_argument if no covariance is set.
   */
  template <class PER_COLUMN>
  void drawNoise(size_t horizon, const PER_COLUMN& perColumn) const {
    if (cholesky_.size() == 0) {
      throw std::invalid_argument(
          "GaussianNoiseSampler::sampleNoise: the covariance is not set.");
    }
    const Eigen::Index dimension = cholesky_.rows();
    noise_.resize(dimension, numSamples_ * horizon);

    const std::uint64_t step = stepCounter_;
    const auto drawSample = [&](size_t i) {
      SplitMix64 generator(sampleSeed(seed_, step, i));
      std::normal_distribution<double> standardNormal(0.0, 1.0);
      for (size_t k = 0; k < horizon; ++k) {
        const Eigen::Index column = i * horizon + k;
        for (Eigen::Index row = 0; row < dimension; ++row) {
          noise_(row, column) = standardNormal(generator);
        }
        perColumn(column, k);
      }
    };
#ifdef GTSAM_USE_TBB
    tbb::parallel_for(size_t(0), numSamples_, drawSample);
#else
    for (size_t i = 0; i < numSamples_; ++i) drawSample(i);
#endif
    ++stepCounter_;
  }

  /**
   * Lower Cholesky factor L of a covariance, with covariance = L L^T, or an
   * empty matrix if the covariance is empty.
   * @throws std::invalid_argument if the covariance is not square or not
   * positive definite.
   */
  static Matrix lowerCholesky(const Matrix& covariance) {
    if (covariance.size() == 0) return Matrix();
    if (covariance.rows() != covariance.cols()) {
      throw std::invalid_argument(
          "GaussianNoiseSampler: the covariance must be square.");
    }
    const Eigen::LLT<Matrix> cholesky(covariance);
    if (cholesky.info() != Eigen::Success) {
      throw std::invalid_argument(
          "GaussianNoiseSampler: the covariance must be positive definite.");
    }
    return cholesky.matrixL();
  }

  /// Golden-ratio increment of SplitMix64.
  static constexpr std::uint64_t kSplitMix64Increment = 0x9e3779b97f4a7c15ULL;

  /**
   * The SplitMix64 output mix (Steele, Lea and Flood, "Fast splittable
   * pseudorandom number generators", OOPSLA 2014), with the constants of
   * Vigna's splitmix64.c. It is a bijection on 64-bit values.
   */
  static constexpr std::uint64_t splitMix64Mix(std::uint64_t z) {
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
  }

  /**
   * SplitMix64 generator, one per sample. It meets the
   * UniformRandomBitGenerator requirements, so std::normal_distribution can
   * draw from it.
   */
  class SplitMix64 {
   public:
    using result_type = std::uint64_t;

    /// Start the generator at the given seed.
    explicit SplitMix64(std::uint64_t seed) : state_(seed) {}

    /// Smallest value the generator returns.
    static constexpr result_type min() { return 0; }

    /// Largest value the generator returns.
    static constexpr result_type max() { return ~result_type(0); }

    /// Next 64-bit value.
    result_type operator()() {
      return splitMix64Mix(state_ += kSplitMix64Increment);
    }

   private:
    std::uint64_t state_;  ///< Generator state.
  };

  /**
   * The seed of sample `sample` at sampling step `step`. The three values
   * are mixed in turn, so different (seed, step, sample) triples give
   * unrelated seeds.
   */
  static std::uint64_t sampleSeed(std::uint64_t seed, std::uint64_t step,
                                  std::uint64_t sample) {
    const auto mix = [](std::uint64_t value) {
      return splitMix64Mix(value + kSplitMix64Increment);
    };
    return mix(mix(mix(seed) ^ step) ^ sample);
  }

  const Matrix cholesky_;      ///< Cholesky factor L of the covariance.
  const size_t numSamples_;    ///< Number of samples K.
  const std::uint64_t seed_;   ///< Seed of the per-sample generators.

  mutable std::uint64_t stepCounter_ = 0;  ///< Sampling calls so far.
  mutable Matrix noise_;            ///< Z: standard normal noise, M x (K*H).
  mutable Matrix scaledNoise_;      ///< E = L Z, M x (K*H).
  mutable Matrix sampledControls_;  ///< u + E, M x (K*H).
};

}  // namespace gtsam
