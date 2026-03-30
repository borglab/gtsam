
#include <gtsam/sam/RISAMGraduationScheduler.h>

#include <limits>

namespace gtsam {

/* ************************************************************************* */
double SIGScheduler::updateMu(const double& mu, const double& residual,
                              const size_t& update_count) const {
  return this->mu_update_strat_(mu, residual, update_count);
}

/* ************************************************************************* */
double SIGScheduler::updateMuInit(const double& mu,
                                  const bool is_inlier) const {
  double new_mu = mu + (is_inlier ? -mu_init_increment_ : mu_init_increment_);
  return std::clamp(new_mu, mu_init_, convergence_thresh_);
}

/* ************************************************************************* */
bool SIGScheduler::isMuConverged(const double& mu) const {
  return mu >= convergence_thresh_;
}

/* ************************************************************************* */
double SIGScheduler::muUpdateMcGann2023(const double& mu,
                                        const double& residual,
                                        const size_t& update_count) {
  return std::min(1.0, mu + (mu + 0.1) * 1.2);
}

/* ************************************************************************* */
double SIGScheduler::muUpdateStable(const double& mu, const double& residual,
                                    const size_t& update_count) {
  // Conditional Arithmetic Series: mu[t+1] = mu[t] + 0.5 if (t==0), mu[t] +
  // 0.4 if (t==1), else mu[t] + 0.05
  // Authors attempt to build the most stable method possible from years of
  // using the algorithm.
  if (update_count == 0) return std::min(1.0, mu + 0.5);
  if (update_count == 1) return std::min(1.0, mu + 0.4);
  return std::min(1.0, mu + 0.05);
}

/* ************************************************************************* */
double ScaledScheduler::updateMu(const double& mu, const double& residual,
                                 const size_t& update_count) const {
  return std::clamp(mu_scale_ * mu, convergence_thresh_, mu_init_);
}

/* ************************************************************************* */
double ScaledScheduler::updateMuInit(const double& mu_init,
                                     const bool is_inlier) const {
  double new_mu_init =
      mu_init * (is_inlier ? 1.0 / mu_init_scale_ : mu_init_scale_);
  return std::clamp(new_mu_init, convergence_thresh_, mu_init_);
}

/* ************************************************************************* */
bool ScaledScheduler::isMuConverged(const double& mu) const {
  return mu <= convergence_thresh_;
}

}  // namespace gtsam