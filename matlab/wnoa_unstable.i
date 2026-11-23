
/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file wnoa_unstable.i
 * @brief MATLAB bindings for WNOA interpolation with AugmentedPoint.
 * @author Oluwatodimu Afolayan
 */

// MATLAB-only constructor bridge for the unstable WNOA interpolation factor.
// Kept separate from gtsam_unstable.i so Python bindings remain unchanged.
namespace gtsam {

#include <MatlabWnoaInterpFactor.h>

virtual class MatlabWnoaInterpFactorAugmentedPoint3
    : gtsam::NoiseModelFactor {
  MatlabWnoaInterpFactorAugmentedPoint3(
      const gtsam::NoiseModelFactor& inner_factor,
      const gtsam::StateData& left,
      const gtsam::StateData& interpolated,
      const gtsam::StateData& right,
      const gtsam::Vector& q_psd_diag,
      bool fixed_noise_model = false,
      bool precomp_interp_mats = true);
};

}  // namespace gtsam
