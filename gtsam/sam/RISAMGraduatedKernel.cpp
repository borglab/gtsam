#include "risam/GraduatedKernel.h"

#include <limits>

#include "risam/Utilities.h"
namespace risam {

/*********************************************************************************************************************/
double SIGKernel::error(const double &residual, const double &mu) const {
  double r2 = residual * residual;
  double c2 = params_.shape_param * params_.shape_param;
  return 0.5 * (c2 * r2) / (c2 + pow(r2, mu));
}

/*********************************************************************************************************************/
double SIGKernel::weight(const double &residual, const double &mu) const {
  double r2 = residual * residual;
  double c2 = params_.shape_param * params_.shape_param;
  double sqrt_denom = c2 + pow(r2, mu);
  return (c2 * (c2 + pow(r2, mu) * (1 - mu))) / (sqrt_denom * sqrt_denom);
}

/*********************************************************************************************************************/
double SIGKernel::updateMu(const double &mu, const double &residual, const size_t &update_count) const {
  if (params_.mu_update_strat == MuUpdateStrategy::MCGANN_2023) {
    // Mu update empirically discovered and presented in the orig riSAM paper
    return std::min(1.0, mu + (mu + 0.1) * 1.2);
  } else if (params_.mu_update_strat == MuUpdateStrategy::SIMPLE_HUBER) {
    // Simple mu update of [mu_init, 0.5, 1.0] where 0.5 results in approximately the huber kernel
    return mu < 0.5 ? 0.5 : 1.0;
  } else if (params_.mu_update_strat == MuUpdateStrategy::KANG_CODE_2023) {
    // 3 Step process [mu_init, mu*, 1] where mu* is the inflection point for the current residual
    if (!params_.kang_residual_threshold) {
      throw std::runtime_error("mu_update_strat == KANG_CODE_2023 but no kang_residual_threshold provided in params");
    }
    if (update_count == 0 && residual > params_.shape_param / std::sqrt(3) &&
        residual < (*params_.kang_residual_threshold)) {
      double mu_inflection = bisectionMuSearch(
          [residual, this](double new_mu) { return secondDerivative(residual, new_mu); }, 0.5, 1.0, 1e-9);
      // If the inflection point is less than the current mu_init skip to mu_final
      return mu_inflection <= mu ? 1.0 : mu_inflection;
    } else {
      // If update_count > 0 then we have already run the mu* update, so return 1.0
      // If r < c/sqrt(3) then the result will be > 1.0 so skip search and return 1.0
      // If r > kang_residual_threshold after the convex iter then this measurement is most def an outlier so go to 1.0
      // Note: This final condition is present in the code, but not in the paper by Kang et al.
      return 1.0;
    }
  } else if (params_.mu_update_strat == MuUpdateStrategy::KANG_PAPER_2023) {
    // 3 Step process [mu_init, mu*, 1] where mu* is the inflection point for the current residual
    if (update_count == 0 && residual > params_.shape_param / std::sqrt(3)) {
      double mu_inflection = bisectionMuSearch(
          [residual, this](double new_mu) { return secondDerivative(residual, new_mu); }, 0.5, 1.0, 1e-9);
      // If the inflection point is less than the current mu_init skip to mu_final
      return mu_inflection <= mu ? 1.0 : mu_inflection;
    } else {
      // If update_count > 0 then we have already run the mu* update, so return 1.0
      // If r < c/sqrt(3) then the result will be > 1.0 so skip search and return 1.0
      return 1.0;
    }
  } else if (params_.mu_update_strat == MuUpdateStrategy::ONESTEP) {
    // return mu_final after running with mu_init
    return 1.0;
  } else if (params_.mu_update_strat == MuUpdateStrategy::HALVES) {
    // Arithmetic Series: mu[t+1] = mu[t] + (1/2)
    return std::min(1.0, mu + 0.5);
  } else if (params_.mu_update_strat == MuUpdateStrategy::THIRDS) {
    // Arithmetic Series: mu[t+1] = mu[t] + (1/3)
    return std::min(1.0, mu + (1.0 / 3.0));
  } else if (params_.mu_update_strat == MuUpdateStrategy::FOURTHS) {
    // Arithmetic Series: mu[t+1] = mu[t] + (1/4)
    return std::min(1.0, mu + 0.25);
  } else if (params_.mu_update_strat == MuUpdateStrategy::ASYM_FOURTHS) {
    // Conditional Arithmetic Series: mu[t+1] = mu[t] + 0.5 if (t==0) else mu[t] + 0.25
    if (update_count == 0) return std::min(1.0, mu + 0.5);
    return std::min(1.0, mu + 0.25);
  } else if (params_.mu_update_strat == MuUpdateStrategy::NONCONVEX) {
    // Conditional Arithmetic Series: mu[t+1] = mu[t] + 0.7 if (t==0) else mu[t] + 0.2
    if (update_count == 0) return std::min(1.0, mu + 0.7);
    return std::min(1.0, mu + 0.2);
  } else if (params_.mu_update_strat == MuUpdateStrategy::STABLE) {
    // Conditional Arithmetic Series: mu[t+1] = mu[t] + 0.5 if (t==0), mu[t] + 0.4 if (t==1), else mu[t] + 0.05
    // My attempt to build the most stable method possible given lessons learned from multi-robot conditions
    if (update_count == 0) return std::min(1.0, mu + 0.5);
    if (update_count == 1) return std::min(1.0, mu + 0.4);
    return std::min(1.0, mu + 0.05);
  } else {
    throw std::runtime_error("updateMu: requested MuUpdateStrategy is not implemented.");
  }
}

/*********************************************************************************************************************/
double SIGKernel::incrementMuInit(const double &mu) const {
  if (params_.mu_init_inc_strat == MuInitIncrementStrategy::MCGANN_2023) {
    // Mu init update Empirically discovered and presented in the orig riSAM paper [Non-Symmetric]
    return std::min(1.0, mu + (mu + 0.1) * 1.2);
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_3) {
    // Simple update with 3 steps from init to final
    return std::min(1.0, mu + (1.0 / 3.0));
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_4) {
    // Simple update with 4 steps from init to final
    return std::min(1.0, mu + 0.25);
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_5) {
    // Simple update with 5 steps from init to final
    return std::min(1.0, mu + 0.2);
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_10) {
    // Simple update with 10 steps from init to final
    return std::min(1.0, mu + 0.1);
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_20) {
    // Simple update with 20 steps from init to final
    return std::min(1.0, mu + 0.05);
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_50) {
    // Simple update with 50 steps from init to final
    return std::min(1.0, mu + 0.02);
  } else {
    throw std::runtime_error("incrementMuInit: requested MuInitIncrementStrategy is not implemented.");
  }
}

/*********************************************************************************************************************/
double SIGKernel::incrementMuInitInv(const double &mu) const {
  if (params_.mu_init_inc_strat == MuInitIncrementStrategy::MCGANN_2023) {
    // Mu init update Empirically discovered and presented in the orig riSAM paper [Non-Symmetric]
    return std::max(0.0, mu - 0.1);
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_3) {
    // Simple Update with 3 steps from init to final
    return std::max(0.0, mu - (1.0 / 3.0));
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_4) {
    // Simple Update with 4 steps from init to final
    return std::max(0.0, mu - (1.0 / 4.0));
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_5) {
    // Simple update with 5 steps from init to final
    return std::max(0.0, mu - 0.2);
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_10) {
    // Simple update with 10 steps from init to final
    return std::max(0.0, mu - 0.1);
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_20) {
    // Simple update with 20 steps from init to final
    return std::max(0.0, mu - 0.05);
  } else if (params_.mu_init_inc_strat == MuInitIncrementStrategy::EQUAL_50) {
    // Simple update with 50 steps from init to final
    return std::max(0.0, mu - 0.02);
  } else {
    throw std::runtime_error("incrementMuInitInv: requested MuInitIncrementStrategy is not implemented.");
  }
}

/*********************************************************************************************************************/
bool SIGKernel::isMuConverged(const double &mu) const { return mu >= convergence_thresh_; }

/*********************************************************************************************************************/
double SIGKernel::shapeParamFromInfThresh(double influence_thresh, size_t dof, double chi2_outlier_thresh) {
  const double outlier_residual_thresh =
      boost::math::quantile(boost::math::chi_squared_distribution<double>(dof), chi2_outlier_thresh);
  // Equation from taking derivative of SIGKernel setting equal to influence_thresh and solving for mu=1
  const double t1 = std::sqrt(2 * influence_thresh * std::pow(outlier_residual_thresh, 5));
  const double t2 = influence_thresh * std::pow(outlier_residual_thresh, 2);
  const double t3 = influence_thresh - 2 * outlier_residual_thresh;
  return std::sqrt(-((t1 + t2) / t3));
}

/*********************************************************************************************************************/
double SIGKernel::shapeParamFromChi2(size_t dof, double chi2_threshold) {
  return boost::math::quantile(boost::math::chi_squared_distribution<double>(dof), chi2_threshold);
}

/*********************************************************************************************************************/
double SIGKernel::secondDerivative(const double &r, const double &mu) const {
  // Setup helpful values
  double c2 = params_.shape_param * params_.shape_param;
  double c4 = c2 * c2;
  double c6 = c4 * c2;

  // Calculate
  double numerator = c6 -                                                    // Constant
                     (c4 * (mu + 2) * (2 * mu - 1) * std::pow(r, 2 * mu)) +  // term 1
                     (c2 * (mu - 1) * (2 * mu - 1) * std::pow(r, 4 * mu));   // term 2
  double denominator = std::pow(c2 + std::pow(r, 2 * mu), 3);

  return numerator / denominator;
}
}  // namespace risam