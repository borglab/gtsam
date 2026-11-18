#pragma once

#include <gtsam/nonlinear/WnoaInterpFactor.h>
#include <gtsam_unstable/geometry/AugmentedPoint.h>

#include <memory>
#include <set>
#include <stdexcept>

namespace gtsam {

// MATLAB-only constructor bridge for the AugmentedPoint3 WNOA factor.
// MATLAB's generated wrappers cannot directly construct std::set<StateData>
// or pass NoiseModelFactor::shared_ptr as a constructor argument.
class MatlabWnoaInterpFactorAugmentedPoint3
    : public WnoaInterpFactor<AugmentedPoint<3>> {
 private:
  using Base = WnoaInterpFactor<AugmentedPoint<3>>;

  static NoiseModelFactor::shared_ptr cloneFactor(
      const NoiseModelFactor& factor) {
    auto copy = std::dynamic_pointer_cast<NoiseModelFactor>(factor.clone());
    if (!copy) {
      throw std::invalid_argument(
          "MATLAB WNOA interpolation requires a cloneable NoiseModelFactor");
    }
    return copy;
  }

  static Eigen::Matrix<double, 3, 1> checkedPSD(const Vector& q_psd_diag) {
    if (q_psd_diag.size() != 3) {
      throw std::invalid_argument(
          "MATLAB WNOA interpolation expects a 3-element PSD vector");
    }
    return q_psd_diag;
  }

 public:
  MatlabWnoaInterpFactorAugmentedPoint3(
      const NoiseModelFactor& inner_factor, const StateData& left,
      const StateData& interpolated, const StateData& right,
      const Vector& q_psd_diag, bool fixed_noise_model = false,
      bool precomp_interp_mats = true)
      : Base(cloneFactor(inner_factor), std::set<StateData>{left, right},
             std::set<StateData>{interpolated}, checkedPSD(q_psd_diag),
             fixed_noise_model, precomp_interp_mats) {}
};

}  // namespace gtsam
