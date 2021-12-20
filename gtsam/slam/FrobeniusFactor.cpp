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

//******************************************************************************
SharedNoiseModel
ConvertNoiseModel(const SharedNoiseModel &model, size_t d, bool defaultToUnit) {
  double sigma = 1.0;

  if (model != nullptr) {
    const auto &robust = boost::dynamic_pointer_cast<noiseModel::Robust>(model);
    Vector sigmas;
    if (robust) {
      sigmas = robust->noise()->sigmas();
    } else {
      sigmas = model->sigmas();
    }

    size_t n = sigmas.size();
    if (n == 1) {
      sigma = sigmas(0); // Rot2
      goto exit;
    }
    else if (n == 3 || n == 6) {
      sigma = sigmas(2); // Pose2, Rot3, or Pose3
      if (sigmas(0) != sigma || sigmas(1) != sigma) {
        if (!defaultToUnit) {
          throw std::runtime_error("Can only convert isotropic rotation noise");
        }
      }
      goto exit;
    }
    if (!defaultToUnit) {
      throw std::runtime_error("Can only convert Pose2/Pose3 noise models");
    }
  }
  exit:
  auto isoModel = noiseModel::Isotropic::Sigma(d, sigma);
  const auto &robust = boost::dynamic_pointer_cast<noiseModel::Robust>(model);
  if (robust) {
    return noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345), isoModel);
  } else {
    return isoModel;
  }
}

//******************************************************************************

} // namespace gtsam
