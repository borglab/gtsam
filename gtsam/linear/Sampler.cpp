/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Sampler.cpp
 * @brief sampling from a diagonal NoiseModel
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

#include <gtsam/linear/Sampler.h>
namespace gtsam {

/* ************************************************************************* */
Sampler::Sampler(const noiseModel::Diagonal::shared_ptr& model,
                 uint_fast64_t seed)
    : model_(model), generator_(seed) {}

/* ************************************************************************* */
Sampler::Sampler(const Vector& sigmas, uint_fast64_t seed)
    : model_(noiseModel::Diagonal::Sigmas(sigmas, true)), generator_(seed) {}

/* ************************************************************************* */
Vector Sampler::sampleDiagonal(const Vector& sigmas) const {
  size_t d = sigmas.size();
  Vector result(d);
  for (size_t i = 0; i < d; i++) {
    double sigma = sigmas(i);

    // handle constrained case separately
    if (sigma == 0.0) {
      result(i) = 0.0;
    } else {
      typedef std::normal_distribution<double> Normal;
      Normal dist(0.0, sigma);
      result(i) = dist(generator_);
    }
  }
  return result;
}

/* ************************************************************************* */
Vector Sampler::sample() const {
  assert(model_.get());
  const Vector& sigmas = model_->sigmas();
  return sampleDiagonal(sigmas);
}

/* ************************************************************************* */
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
Sampler::Sampler(uint_fast64_t seed) : generator_(seed) {}

Vector Sampler::sampleNewModel(
    const noiseModel::Diagonal::shared_ptr& model) const {
  assert(model.get());
  const Vector& sigmas = model->sigmas();
  return sampleDiagonal(sigmas);
}
#endif
/* ************************************************************************* */

}  // namespace gtsam
