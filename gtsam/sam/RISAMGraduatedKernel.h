/** @brief Interface for a graduated robust kernel.
 * That is a robust kernel \rho(r_i, \mu) that varies from highly convex (quadratic) when \mu = \mu_{init} to non-convex
 * (robust cost) as \mu trends towards \mu_{final}
 *
 *
 * Citations:
 * [2] - W. Kang et al. "Efficient Graduated Non-Convexity for Pose Graph Optimization", International Conference on
 * Control, Automation and Systems (ICCAS), 2024
 *
 *  @author Dan McGann
 *  @date Mar 2022
 */
#pragma once
#include <gtsam/base/FastVector.h>
#include <gtsam/base/Matrix.h>

#include <boost/math/distributions/chi_squared.hpp>
#include <optional>

namespace risam {
class GraduatedKernel {
  /** TYPES **/
 public:
  typedef boost::shared_ptr<GraduatedKernel> shared_ptr;

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
  virtual double error(const double &residual, const double &mu) const = 0;

  /** @brief Computes the weight of the graduated robust cost function
   * weight = w(r_i)
   *
   * @param residual: r_i the current (whitened) residual
   * @param mu: \mu the current convexity parameter
   */
  virtual double weight(const double &residual, const double &mu) const = 0;

  /** @brief Weights a linearized system
   * Weight results in the solving of: w_i * r_i^2 = \sqrt(w_i) (A_i x + b_i) = \sqrt(w_i)* A_i x + \sqrt(w_i) * b_i
   *
   * @param A: Whitened linear system from a factor (\Sigma^{-0.5} H_i)
   * @param b: Whitened linear system error from a factor (0.5 * \Sigma^(-0.5) ||h_i(x_i) - z_i||^2 = 0.5 * r_i^2)
   */
  void weightSystem(std::vector<gtsam::Matrix> &A, gtsam::Vector &b, const double &residual, const double &mu) const {
    double sqrt_weight = sqrt(weight(residual, mu));
    for (gtsam::Matrix &Aj : A) {
      Aj *= sqrt_weight;
    }
    b *= sqrt_weight;
  }

  /// @brief Returns the value of \mu_{init} for this graduated kernel
  double muInit() const { return mu_init_; }

  /// @brief Returns the next value of \mu for this graduated kernel
  /// @param mu: The current value of mu
  /// @param residual: The current residual of the factor
  /// @param update_count: The number of mu updates during this graduated solve 0 = just made convex
  virtual double updateMu(const double &mu, const double &residual, const size_t &update_count) const = 0;

  /// @brief Returns an incremented mu_init, used to decrease the influence of known outliers
  virtual double incrementMuInit(const double &mu_init) const = 0;

  /// @brief Returns the previous value of mu_init for this graduated kernel
  virtual double incrementMuInitInv(const double &mu) const = 0;

  /// @brief Returns true iff the value of \mu has converged to a non-convex state
  virtual bool isMuConverged(const double &mu) const = 0;
};

/* ************************************************************************* */
class SIGKernel : public GraduatedKernel {
  /** My version of the Graduated version of the Geman-McClure kernel */

  /** TYPES **/
 public:
  /// @brief Shortcut for shared pointer
  typedef boost::shared_ptr<SIGKernel> shared_ptr;

  /// @brief Strategies for updating Mu between GNC iterations of riSAM
  enum class MuUpdateStrategy {
    MCGANN_2023,      // Geometric Series:  mu[t+1] = mu[t] + (mu[t] + 0.1) * 1.2
    SIMPLE_HUBER,     // Fixed Pattern [mu_init, 0.5, 1.0]
    KANG_CODE_2023,   // Sequence: [mu_init, mu*, 1.0] with early outlier rejection for r > kang_residual_threshold [2]
    KANG_PAPER_2023,  // Sequence: [mu_init, mu*, 1.0] (no early outlier rejection was presented in paper) [2]
    ONESTEP,          // Jump to mu=1.0 after initial convex step
    HALVES,           // Arithmetic Series: mu[t+1] = mu[t] + (1/2)
    THIRDS,           // Arithmetic Series: mu[t+1] = mu[t] + (1/3)
    FOURTHS,          // Arithmetic Series: mu[t+1] = mu[t] + (1/4)
    ASYM_FOURTHS,     // Conditional Arithmetic Series: mu[t+1] = mu[t] + 0.5 if (t==0) else mu[t] + 0.25
    NONCONVEX,        // Conditional Arithmetic Series: mu[t+1] = mu[t] + 0.7 if (t==0) else mu[t] + 0.2
    STABLE  // Conditional Arithmetic Series: mu[t+1] = mu[t] + 0.5 if (t==0), mu[t] + 0.4 if (t==1), else mu[t] + 0.05
  };
  /// @brief Strategies for update Mu_init after convergence
  enum class MuInitIncrementStrategy { MCGANN_2023, EQUAL_3, EQUAL_4, EQUAL_5, EQUAL_10, EQUAL_20, EQUAL_50 };

  struct Parameters {
    /// @brief The shape parameter of the GM kernel.
    double shape_param{1};
    /// @brief The update strategy to use for mu updates
    MuUpdateStrategy mu_update_strat{MuUpdateStrategy::MCGANN_2023};
    /// @brief The update strategy to use for mu_init increments
    MuInitIncrementStrategy mu_init_inc_strat{MuInitIncrementStrategy::MCGANN_2023};

    /// @brief The residual threshold use to skip to mu=1.0 in the provided for [2]
    /// Required only if mu_update_strat == KANG_CODE_2023
    std::optional<double> kang_residual_threshold{std::nullopt};

    /// @brief Default Constructor
    explicit Parameters() = default;
    /// @brief Parameterized Constructor
    Parameters(double shape_param, MuUpdateStrategy mu_update_strat, MuInitIncrementStrategy mu_init_inc_strat,
               std::optional<double> kang_residual_threshold = std::nullopt)
        : shape_param(shape_param),
          mu_update_strat(mu_update_strat),
          mu_init_inc_strat(mu_init_inc_strat),
          kang_residual_threshold(kang_residual_threshold) {}
    /// @brief Static Constructor to work around GCC Bug
    static Parameters Default() { return Parameters(); };
  };

  /** FIELDS **/
 public:
  const Parameters params_;

  /** @brief Params Constructor
   * @param params: The parameters to configure the kernel
   */
  SIGKernel(const Parameters params = Parameters::Default()) : GraduatedKernel(0.0, 1.0), params_(params) {};

  /** @brief Individual Parameter Constructor
   * @param shape_param: The shape parameter for the kernel
   * @param mu_update_strat: The update strategy to use for mu updates
   * @param mu_init_inc_strat: The update strategy to use for mu_init increments
   */
  SIGKernel(double shape_param, MuUpdateStrategy mu_update_strat, MuInitIncrementStrategy mu_init_inc_strat,
            std::optional<double> kang_residual_threshold = std::nullopt)
      : GraduatedKernel(0.0, 1.0),
        params_(Parameters(shape_param, mu_update_strat, mu_init_inc_strat, kang_residual_threshold)) {};

  /** Interface **/
 public:
  /// @brief @see GraduatedKernel
  double error(const double &residual, const double &mu) const override;
  /// @brief @see GraduatedKernel
  double weight(const double &residual, const double &mu) const override;
  /// @brief @see GraduatedKernel
  double updateMu(const double &mu, const double &residual, const size_t &update_count) const override;
  /// @brief @see GraduatedKernel
  double incrementMuInit(const double &mu) const override;
  /// @brief @see GraduatedKernel
  double incrementMuInitInv(const double &mu) const override;
  /// @brief @see GraduatedKernel
  bool isMuConverged(const double &mu) const override;

  /** @brief Computes a shape param based on an an influence threshold for outliers
   * Computes a shape param such that an outlier (any measurement with residual equal to or greater than the
   * chi2_outlier_threshold) will have an "influence" (derivative of kernel at mu=1) less than or equal to the
   * influence_threshold: d/dr(\rho(r^2)) < influence_thresh
   * @param influence_thresh - The max influence permited by an outlier
   * @param dof - The degrees of freedom of the corresponding measurement (use for chi2)
   * @param chi2_outlier_thresh - The threshold for outlier (i.e. 0.95 = any measurement with residual greater than 95%
   * of expected measurements is an outlier)
   * @returns The shape param such that outliers will have inf <= influence_thresh
   */
  static double shapeParamFromInfThresh(double influence_thresh, size_t dof, double chi2_outlier_thresh);

  /** @brief Computes a shape param based on dimensionality and a chi2_threshold
   * Computes a shape param c = chi2.inv_cdf(dof, thresh)
   * @param dof - The degrees of freedom of the corresponding measurement
   * @param chi2_threshold - The the threshold at which to compute the shape param value
   * @returns The shape param that scales based on dimensionality of the measurement
   */
  static double shapeParamFromChi2(size_t dof, double chi2_threshold);

  /** HELPERS **/
 protected:
  /// @brief Second derivative of the SIG Kernel wrt r
  double secondDerivative(const double &r, const double &mu) const;
};
}  // namespace risam