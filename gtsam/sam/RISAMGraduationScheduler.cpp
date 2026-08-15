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
                                     const size_t& update_count) const {
  return this->mu_update_strat_(mu, residual, update_count);
}

/* ************************************************************************* */
double GraduationScheduler::updateMuInit(const double& mu,
                                         const bool is_inlier) const {
  double new_mu = mu + (is_inlier ? -mu_init_increment_ : mu_init_increment_);
  return std::clamp(new_mu, mu_init_, convergence_thresh_);
}

/* ************************************************************************* */
bool GraduationScheduler::isMuConverged(const double& mu) const {
  return mu >= convergence_thresh_;
}

/* ************************************************************************* */
double GraduationScheduler::muUpdateMcGann2023(const double& mu,
                                               const double& residual,
                                               const size_t& update_count) {
  return std::min(1.0, mu + (mu + 0.1) * 1.2);
}

/* ************************************************************************* */
double GraduationScheduler::muUpdateStable(const double& mu,
                                           const double& residual,
                                           const size_t& update_count) {
  // Conditional Arithmetic Series: mu[t+1] = mu[t] + 0.5 if (t==0), mu[t] +
  // 0.4 if (t==1), else mu[t] + 0.05
  // Authors attempt to build the most stable method possible from years of
  // using the algorithm.
  if (update_count == 0) return std::min(1.0, mu + 0.5);
  if (update_count == 1) return std::min(1.0, mu + 0.4);
  return std::min(1.0, mu + 0.05);
}

}  // namespace gtsam
