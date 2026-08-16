/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <gtsam/sam/RISAMGraduationScheduler.h>

#include <algorithm>

namespace gtsam {

/* ************************************************************************* */
double GraduationScheduler::updateMu(const double& mu, const double& residual,
                                     const size_t& updateCount) const {
  return this->muUpdateStrategy_(mu, residual, updateCount);
}

/* ************************************************************************* */
double GraduationScheduler::updateMuInit(const double& muInit,
                                         const bool isInlier) const {
  double newMuInit = muInit + (isInlier ? -muInitIncrement_ : muInitIncrement_);
  return std::clamp(newMuInit, muInit_, convergenceThreshold_);
}

/* ************************************************************************* */
bool GraduationScheduler::isMuConverged(const double& mu) const {
  return mu >= convergenceThreshold_;
}

/* ************************************************************************* */
double GraduationScheduler::MuUpdateMcGann2023(const double& mu,
                                               const double& residual,
                                               const size_t& updateCount) {
  return std::min(1.0, mu + (mu + 0.1) * 1.2);
}

/* ************************************************************************* */
double GraduationScheduler::MuUpdateStable(const double& mu,
                                           const double& residual,
                                           const size_t& updateCount) {
  // Conditional Arithmetic Series: mu[t+1] = mu[t] + 0.5 if (t==0), mu[t] +
  // 0.4 if (t==1), else mu[t] + 0.05
  // Authors attempt to build the most stable method possible from years of
  // using the algorithm.
  if (updateCount == 0) return std::min(1.0, mu + 0.5);
  if (updateCount == 1) return std::min(1.0, mu + 0.4);
  return std::min(1.0, mu + 0.05);
}

}  // namespace gtsam
