
#include <gtsam/sam/RISAMGraduatedKernel.h>

#include <limits>

namespace gtsam {

/* ************************************************************************* */
double SIGKernel::error(const double& residual, const double& mu) const {
  double r2 = residual * residual;
  double c2 = this->shape_param * this->shape_param;
  return 0.5 * (c2 * r2) / (c2 + pow(r2, mu));
}

/* ************************************************************************* */
double SIGKernel::weight(const double& residual, const double& mu) const {
  double r2 = residual * residual;
  double c2 = this->shape_param * this->shape_param;
  double sqrt_denom = c2 + pow(r2, mu);
  return (c2 * (c2 + pow(r2, mu) * (1 - mu))) / (sqrt_denom * sqrt_denom);
}

/* ************************************************************************* */
double SIGKernel::updateMu(const double& mu, const double& residual,
                           const size_t& update_count) const {
  return this->mu_update_strat(mu, residual, update_count);
}

/* ************************************************************************* */
double SIGKernel::incrementMuInit(const double& mu) const {
  return std::min(1.0, mu + this->mu_init_increment);
}

/* ************************************************************************* */
double SIGKernel::incrementMuInitInv(const double& mu) const {
  return std::max(0.0, mu - this->mu_init_increment);
}

/* ************************************************************************* */
bool SIGKernel::isMuConverged(const double& mu) const {
  return mu >= convergence_thresh_;
}

/* ************************************************************************* */
double SIGKernel::shapeParamFromInfThresh(double influence_thresh, size_t dof,
                                          double chi2_outlier_thresh) {
  double outlier_residual_thresh =
      internal::chiSquaredQuantile(dof, chi2_outlier_thresh);
  // Equation from taking derivative of SIGKernel setting equal to
  // influence_thresh and solving for mu=1
  const double t1 =
      std::sqrt(2 * influence_thresh * std::pow(outlier_residual_thresh, 5));
  const double t2 = influence_thresh * std::pow(outlier_residual_thresh, 2);
  const double t3 = influence_thresh - 2 * outlier_residual_thresh;
  return std::sqrt(-((t1 + t2) / t3));
}

/* ************************************************************************* */
double SIGKernel::shapeParamFromChi2(size_t dof, double chi2_threshold) {
  return internal::chiSquaredQuantile(dof, chi2_threshold);
}

/* ************************************************************************* */
double SIGKernel::muUpdateMcGann2023(const double& mu, const double& residual,
                                     const size_t& update_count) {
  return std::min(1.0, mu + (mu + 0.1) * 1.2);
}

/* ************************************************************************* */
double SIGKernel::muUpdateStable(const double& mu, const double& residual,
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