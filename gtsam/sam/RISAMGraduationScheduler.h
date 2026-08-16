/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
#include <stdexcept>

namespace gtsam {

/** @brief Class for graduation scheduling for riSAM
 *
 * Custom Schedulers can be implemented by defining custom Mu update strats.
 * All loss functions are convex when $\mu = 0$ and recover their nonconvex
 * form for $\mu = 1.0$.
 *
 * For riSAM we recommend using GemanMcClure loss with SCALE_INVARIANT
 * graduation scheme and the MuUpdateStable strategy configured by default
 * for this class. Advanced users can explore alternate loss and graduation
 * schedules.
 *
 */
class GTSAM_EXPORT GraduationScheduler {
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
  double muInit_;
  /// @brief The threshold at which to consider mu to be converged
  /// @note $\mu = 1$ is the target robust loss by GTSAM convention
  double convergenceThreshold_;
  /// @brief The update strategy for the sequence $\mu$ values
  MuUpdateStrategy muUpdateStrategy_;
  /// @brief The amount to increment/decrement $\mu_{init}$ if the factor is a
  /// strong inlier/outlier when values converge
  double muInitIncrement_;
  /// @}

  /// @name Public Interface
  /// @{
 public:
  /** @brief Individual Parameter Constructor
   * @param muUpdateStrategy: The update strategy to use for $\mu$ updates
   *    Recommend: MuUpdateStable, Alt: MuUpdateMcGann2023
   *    Recs. to be used with GemanMcClure loss with SCALE_INVARIANT graduation
   * @param muInitIncrement: The amount to increment/decrement $\mu_{init}$
   * @param muInit: The starting value of $\mu$ for factors using this
   *    scheduler, in [0, 1]. 0 is the most convex start, and it also floors
   *    updateMuInit, so a nonzero value never graduates more convex than it.
   * @throws std::invalid_argument if muInit is outside [0, 1].
   */
  GraduationScheduler(MuUpdateStrategy muUpdateStrategy = MuUpdateStable,
                      double muInitIncrement = 0.2, double muInit = 0.0)
      : muInit_(muInit),
        convergenceThreshold_(1.0),
        muUpdateStrategy_(muUpdateStrategy),
        muInitIncrement_(muInitIncrement) {
    if (!(muInit >= 0.0 && muInit <= 1.0)) {
      throw std::invalid_argument(
          "GraduationScheduler: muInit must be in [0, 1].");
    }
  }

  /// @brief Returns the value of $\mu_{init}$ for this graduated
  double muInit() const { return muInit_; }

  /** @brief Returns the next value of $\mu$ for this graduated
   * @param mu: The current value of $\mu$
   * @param residual: The current residual of the factor
   * @param updateCount: The number of mu updates during this solve
   */
  double updateMu(const double& mu, const double& residual,
                  const size_t& updateCount) const;

  /** @brief Returns the next value of $\mu_{init}$ for strong inliers/outliers
   *  - Inliers are updated to have more convex $\mu_{init}$
   *  - Outliers are update to have less convex $\mu_{init}$
   * @param muInit: The current value of $\mu_{init}$
   * @param isInlier: Flag indicating inlier update otherwise an outlier update
   */
  double updateMuInit(const double& muInit, const bool isInlier) const;

  /// @brief Returns true iff the value of $\mu$ has converged
  bool isMuConverged(const double& mu) const;
  /// @}

  /// @name Default $\mu$ Update Strategies
  /// @{
 public:
  /// @brief $\mu$ update sequence empirically discovered and presented in the
  /// orig riSAM paper
  /// @note Adheres to MuUpdateStrategy interface
  static double MuUpdateMcGann2023(const double& mu, const double& residual,
                                   const size_t& updateCount);
  /// @brief More stable $\mu$ Update sequence developed empirically since
  /// algorithm was published
  /// @note Adheres to MuUpdateStrategy interface
  static double MuUpdateStable(const double& mu, const double& residual,
                               const size_t& updateCount);
  /// @}
};
}  // namespace gtsam
