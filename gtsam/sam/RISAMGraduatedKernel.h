/** @brief Interface for a graduated robust kernel.
 * That is a robust kernel \rho(r_i, \mu) that varies from highly convex
 * (quadratic) when \mu = \mu_{init} to non-convex (robust cost) as \mu trends
 * towards \mu_{final}
 *
 *
 *
 *  @author Dan McGann
 *  @date Mar 2022
 */
#pragma once
#include <memory>
#include <gtsam/base/FastVector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/internal/ChiSquaredInverse.h>

#include <optional>

namespace gtsam {

/** @brief Base class for graduated kernels for riSAM
 * Advanced users can write their own kernels by inheriting from this class
 */
class GraduatedKernel {
  /** TYPES **/
 public:
  typedef std::shared_ptr<GraduatedKernel> shared_ptr;

  /** FIELDS **/
 protected:
  /// @brief The initial Value for mu
  const double mu_init_;
  /// @brief The threshold at which to consider mu to be converged
  const double convergence_thresh_;

  /** INTERFACE **/
 public:
  GraduatedKernel(double mu_init, double convergence_thresh)
      : mu_init_(mu_init), convergence_thresh_(convergence_thresh) {}

  /** @brief Computes the graduated robust error of the function
   * Error = \rho(r_i)
   *
   * @param residual: r_i the current (whitened) residual
   * @param mu: \mu the current convexity parameter
   */
  virtual double error(const double& residual, const double& mu) const = 0;

  /** @brief Computes the weight of the graduated robust cost function
   * weight = w(r_i)
   *
   * @param residual: r_i the current (whitened) residual
   * @param mu: \mu the current convexity parameter
   */
  virtual double weight(const double& residual, const double& mu) const = 0;

  /** @brief Weights a linearized system
   * Weight results in the solving of: w_i * r_i^2 = \sqrt(w_i) (A_i x + b_i) =
   * \sqrt(w_i)* A_i x + \sqrt(w_i) * b_i
   *
   * @param A: Whitened linear system from a factor (\Sigma^{-0.5} H_i)
   * @param b: Whitened linear system error from a factor (0.5 * \Sigma^(-0.5)
   * ||h_i(x_i) - z_i||^2 = 0.5 * r_i^2)
   */
  void weightSystem(std::vector<gtsam::Matrix>& A, gtsam::Vector& b,
                    const double& residual, const double& mu) const {
    double sqrt_weight = sqrt(weight(residual, mu));
    for (gtsam::Matrix& Aj : A) {
      Aj *= sqrt_weight;
    }
    b *= sqrt_weight;
  }

  /// @brief Returns the value of \mu_{init} for this graduated kernel
  double muInit() const { return mu_init_; }

  /// @brief Returns the next value of \mu for this graduated kernel
  /// @param mu: The current value of mu
  /// @param residual: The current residual of the factor
  /// @param update_count: The number of mu updates during this graduated solve
  /// 0 = just made convex
  virtual double updateMu(const double& mu, const double& residual,
                          const size_t& update_count) const = 0;

  /// @brief Returns an incremented mu_init, used to decrease the influence of
  /// known outliers
  virtual double incrementMuInit(const double& mu_init) const = 0;

  /// @brief Returns the previous value of mu_init for this graduated kernel
  virtual double incrementMuInitInv(const double& mu) const = 0;

  /// @brief Returns true iff the value of \mu has converged to a non-convex
  /// state
  virtual bool isMuConverged(const double& mu) const = 0;
};

/* ************************************************************************* */
/** @brief Implementation of the Scale Invariant Graduated Kernel
 * Based on the Geman-McClure Kernel this kernel has the mathematical form:
 *
 * \rho(r) = 0.5 * \frac{c^2 * r^2}{c^2 + (r^2)^\mu}
 *
 * where c is the shape parameter and r is the residual of the factor
 */
class SIGKernel : public GraduatedKernel {
  /** TYPES **/
 public:
  /// @brief Shortcut for shared pointer
  typedef std::shared_ptr<SIGKernel> shared_ptr;
  /// @brief Function type for mu update sequence
  typedef std::function<double(double, double, size_t)> MuUpdateStrategy;

  /** FIELDS **/
 public:
  /// @brief The shape parameter of the SIG kernel 'c'
  double shape_param;
  /// @brief The update strategy for the sequence mu_ values
  MuUpdateStrategy mu_update_strat;
  /// @brief The amount to increment/decrement mu init if the factor is a strong
  /// inlier/outlier when values converge
  double mu_init_increment;

  /** @brief Individual Parameter Constructor
   * @param shape_param: The shape parameter for the kernel
   * @param mu_update_strat: The update strategy to use for mu updates
   * @param mu_init_increment: The amount to increment/decrement mu init
   */
  SIGKernel(double shape_param,
            MuUpdateStrategy mu_update_strat = muUpdateStable,
            double mu_init_increment = 0.2)
      : GraduatedKernel(0.0, 1.0),
        shape_param(shape_param),
        mu_update_strat(mu_update_strat),
        mu_init_increment(mu_init_increment) {}

  /** Interface **/
 public:
  /// @brief @see GraduatedKernel
  double error(const double& residual, const double& mu) const override;
  /// @brief @see GraduatedKernel
  double weight(const double& residual, const double& mu) const override;
  /// @brief @see GraduatedKernel
  double updateMu(const double& mu, const double& residual,
                  const size_t& update_count) const override;
  /// @brief @see GraduatedKernel
  double incrementMuInit(const double& mu) const override;
  /// @brief @see GraduatedKernel
  double incrementMuInitInv(const double& mu) const override;
  /// @brief @see GraduatedKernel
  bool isMuConverged(const double& mu) const override;

  /** STATIC HELPERS **/
 public:
  /** @brief Computes a shape param based on an an influence threshold for
   * outliers Computes a shape param such that an outlier (any measurement with
   * residual equal to or greater than the chi2_outlier_threshold) will have an
   * "influence" (derivative of kernel at mu=1) less than or equal to the
   * influence_threshold: d/dr(\rho(r^2)) < influence_thresh
   * @param influence_thresh - The max influence permited by an outlier
   * @param dof - The degrees of freedom of the corresponding measurement (use
   * for chi2)
   * @param chi2_outlier_thresh - The threshold for outlier (i.e. 0.95 = any
   * measurement with residual greater than 95% of expected measurements is an
   * outlier)
   * @returns The shape param such that outliers will have inf <=
   * influence_thresh
   */
  static double shapeParamFromInfThresh(double influence_thresh, size_t dof,
                                        double chi2_outlier_thresh);

  /** @brief Computes a shape param based on dimensionality and a chi2_threshold
   * Computes a shape param c = chi2.inv_cdf(dof, thresh)
   * @param dof - The degrees of freedom of the corresponding measurement
   * @param chi2_threshold - The the threshold at which to compute the shape
   * param value
   * @returns The shape param that scales based on dimensionality of the
   * measurement
   */
  static double shapeParamFromChi2(size_t dof, double chi2_threshold);

  /** Default Mu Update Strategies **/
  /// @brief Mu Update sequence empirically discovered and presented in the orig
  /// riSAM paper
  static double muUpdateMcGann2023(const double& mu, const double& residual,
                                   const size_t& update_count);
  /// @brief More stable Mu Update sequence developed empirically since
  /// algorithm was published
  static double muUpdateStable(const double& mu, const double& residual,
                               const size_t& update_count);
};
}  // namespace gtsam