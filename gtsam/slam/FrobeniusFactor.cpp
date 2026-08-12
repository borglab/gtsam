/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   FrobeniusFactor.cpp
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Various factors that minimize some Frobenius norm
 */

#include <gtsam/slam/FrobeniusFactor.h>

using namespace std;

namespace gtsam {

SharedNoiseModel ConvertNoiseModel(const SharedNoiseModel &model,
                                   size_t dimension, bool defaultToUnit) {
  double sigma = 1.0;
  noiseModel::mEstimator::Base::shared_ptr robustLoss;

  if (model != nullptr) {
    // Unwrap robust model if present and remember its robust loss
    SharedNoiseModel baseModel = model;
    if (auto robust = std::dynamic_pointer_cast<noiseModel::Robust>(model)) {
      robustLoss = robust->robust();
      baseModel = robust->noise();
    }

    // Use smart constructor to check for isotropy
    Vector sigmas = baseModel->sigmas();
    auto smartModel = noiseModel::Diagonal::Sigmas(sigmas, true);
    auto isotropic =
        std::dynamic_pointer_cast<noiseModel::Isotropic>(smartModel);

    if (isotropic) {
      sigma = isotropic->sigma();
    } else if (!defaultToUnit) {
      throw std::runtime_error("Can only convert isotropic rotation noise");
    }
  }

  auto isoModel = noiseModel::Isotropic::Sigma(dimension, sigma);
  if (robustLoss) {
    return noiseModel::Robust::Create(robustLoss, isoModel);
  } else {
    return isoModel;
  }
}

//******************************************************************************

}  // namespace gtsam
