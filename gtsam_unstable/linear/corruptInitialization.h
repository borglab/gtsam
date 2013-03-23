/**
 * @file corruptInitialization.h
 *
 * @brief Utilities for using noisemodels to corrupt given initialization value
 * 
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/linear/Sampler.h>

namespace gtsam {

/** given a noisemodel and a measurement, add noise to the measurement */
template<typename T>
T corruptWithNoise(const T& init,
    const noiseModel::Base::shared_ptr& model, Sampler& sampler) {
  Vector n = zero(model->dim());
  noiseModel::Diagonal::shared_ptr
    diag_model = boost::dynamic_pointer_cast<noiseModel::Diagonal>(model);
  if (diag_model)
    n = sampler.sampleNewModel(diag_model);
  return init.retract(n);
}

// specialization for doubles - just adds, rather than retract
template<>
inline double corruptWithNoise<double>(const double& init,
    const noiseModel::Base::shared_ptr& model, Sampler& sampler) {
  double n = 0.0;
  noiseModel::Diagonal::shared_ptr
    diag_model = boost::dynamic_pointer_cast<noiseModel::Diagonal>(model);
  if (diag_model)
    n = sampler.sampleNewModel(diag_model)(0);
  return init + n;
}

// specialization for doubles - just adds, rather than retract
template<>
inline Vector corruptWithNoise<Vector>(const Vector& init,
    const noiseModel::Base::shared_ptr& model, Sampler& sampler) {
  Vector n = zero(init.size());
  noiseModel::Diagonal::shared_ptr
    diag_model = boost::dynamic_pointer_cast<noiseModel::Diagonal>(model);
  if (diag_model)
    n = sampler.sampleNewModel(diag_model);
  return init + n;
}

} // \namespace gtsam




