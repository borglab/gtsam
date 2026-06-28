/** @brief Interface and implementations for GraduationScheduler
 * These are used to define the sequence of problems solved by RISAM.
 * The scheduler defines these problems using the the control parameter $\mu$
 * for a specific Graduated Robust Loss Function.
 *  @author Dan McGann
 *  @date Mar 2022
 */
#pragma once
#include <gtsam/base/FastVector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/internal/ChiSquaredInverse.h>

#include <memory>
#include <optional>

namespace gtsam {

/** @brief Class for graduation scheduling for riSAM
 *
 * Custom Schedulers can be implemented by defining custom Mu update strats.
 * All loss functions are convex when $\mu = 0$ and recover their nonconvex
 * form for $\mu = 1.0$.
 *
 * For riSAM we reccomend using GemanMcClure loss with SCALE_INVARIANT
 * graduation scheme and the muUpdateStable strategy configured by default
 * for this class. Advanced users can explore alternate loss and graduation
 * schedules.
 *
 */
class GraduationScheduler {
  /// @name Types
  /// @{
 public:
  /// @brief Shortcut for shared pointer
  typedef std::shared_ptr<GraduationScheduler> shared_ptr;
  /// @brief Function type for $\mu$ update sequence
  typedef std::function<double(double, double, size_t)> MuUpdateStrategy;
  /// @}

  /// @name Fields
  /// @{
 protected:
  /// @brief The initial value for mu $\mu_{init}$
  const double mu_init_;
  /// @brief The threshold at which to consider mu to be converged
  const double convergence_thresh_;
  /// @brief The update strategy for the sequence $\mu$ values
  MuUpdateStrategy mu_update_strat_;
  /// @brief The amount to increment/decrement $\mu_{init}$ if the factor is a
  /// strong inlier/outlier when values converge
  double mu_init_increment_;
  /// @}

  /// @name Public Interface
  /// @{
 public:
  /** @brief Individual Parameter Constructor
   * @param mu_update_strat: The update strategy to use for $\mu$ updates
   *    Recommend: muUpdateStable, Alt: muUpdateMcGann2023
   *    Recs. to be used with GemanMcClure loss with SCALE_INVARIANT graduation
   * @param mu_init_increment: The amount to increment/decrement $\mu_{init}$
   */
  GraduationScheduler(MuUpdateStrategy mu_update_strat = muUpdateStable,
                      double mu_init_increment = 0.2)
      : mu_init_(0.0),
        convergence_thresh_(1.0),
        mu_update_strat_(mu_update_strat),
        mu_init_increment_(mu_init_increment) {}

  /// @brief Returns the value of $\mu_{init}$ for this graduated
  double muInit() const { return mu_init_; }

  /** @brief Returns the next value of $\mu$ for this graduated
   * @param mu: The current value of $\mu$
   * @param residual: The current residual of the factor
   * @param update_count: The number of mu updates during this solve
   */
  double updateMu(const double& mu, const double& residual,
                  const size_t& update_count) const;

  /** @brief Returns the next value of $\mu_{init}$ for strong inliers/outliers
   *  - Inliers are updated to have more convex $\mu_{init}$
   *  - Outliers are update to have less convex $\mu_{init}$
   * @param mu_init: The current value of $\mu_{init}$
   * @param is_inlier: Flag indicating inlier update otherwise an outlier update
   */
  double updateMuInit(const double& mu_init, const bool is_inlier) const;

  /// @brief Returns true iff the value of $\mu$ has converged
  bool isMuConverged(const double& mu) const;
  /// @}

  /// @name Default $\mu$ Update Strategies
  /// @{
 public:
  /// @brief $\mu$ update sequence empirically discovered and presented in the
  /// orig riSAM paper
  static double muUpdateMcGann2023(const double& mu, const double& residual,
                                   const size_t& update_count);
  /// @brief More stable $\mu$ Update sequence developed empirically since
  /// algorithm was published
  static double muUpdateStable(const double& mu, const double& residual,
                               const size_t& update_count);
  /// @}
};
}  // namespace gtsam
