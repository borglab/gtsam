/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Sampler.h
 * @brief sampling from a NoiseModel
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/linear/NoiseModel.h>

#include <random>

namespace gtsam {

/**
 * Sampling structure that keeps internal random number generators for
 * diagonal distributions specified by NoiseModel
 */
class GTSAM_EXPORT Sampler {
 protected:
  /** noiseModel created at generation */
  noiseModel::Diagonal::shared_ptr model_;

  /** generator */
  mutable std::mt19937_64 generator_;

 public:
  typedef boost::shared_ptr<Sampler> shared_ptr;

  /// @name constructors
  /// @{

  /**
   * Create a sampler for the distribution specified by a diagonal NoiseModel
   * with a manually specified seed
   *
   * NOTE: do not use zero as a seed, it will break the generator
   */
  explicit Sampler(const noiseModel::Diagonal::shared_ptr& model,
                   uint_fast64_t seed = 42u);

  /**
   * Create a sampler for a distribution specified by a vector of sigmas
   * directly
   *
   * NOTE: do not use zero as a seed, it will break the generator
   */
  explicit Sampler(const Vector& sigmas, uint_fast64_t seed = 42u);

  /// @}
  /// @name access functions
  /// @{

  size_t dim() const { return model_->dim(); }

  Vector sigmas() const { return model_->sigmas(); }

  const noiseModel::Diagonal::shared_ptr& model() const { return model_; }

  /// @}
  /// @name basic functionality
  /// @{

  /// sample from distribution
  Vector sample() const;

  /// sample with given random number generator
  static Vector sampleDiagonal(const Vector& sigmas, std::mt19937_64* rng);
  /// @}

 protected:
  /** given sigmas for a diagonal model, returns a sample */
  Vector sampleDiagonal(const Vector& sigmas) const;
};

}  // namespace gtsam
