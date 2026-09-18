/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/** @brief Factor that implements a graduated robust cost function.
 * A Graduated factor overrides a portion of the NoiseModelFactor interface to
 * implement a graduated robust cost function for the factor.
 *
 *  @author Dan McGann
 *  @date Mar 2022
 */
#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/sam/RISAMGraduationScheduler.h>

#include <cmath>

namespace gtsam {

/// @brief Graduated Factor for riSAM base class
class GTSAM_EXPORT GraduatedFactor {
  /// @name Types
  /// @{
 public:
  typedef std::shared_ptr<GraduatedFactor> shared_ptr;
  typedef noiseModel::mEstimator::Base RobustLoss;
  /// @}

  /// @name Fields
  /// @{
 protected:
  /// @brief The robust loss for this factor
  RobustLoss::shared_ptr robustLoss_;
  /// @brief The control param ($\mu$) scheduler for this factor
  GraduationScheduler::shared_ptr scheduler_;

  /// @brief The unique mu control parameter for this factor
  std::shared_ptr<double> mu_;
  /// @}

  /// Befriend RISAM to give access to these protected values
  friend class RISAM;

  /// @name Public Interface
  /// @{
 public:
  /** @brief Constructor
   * @param args: The arguments required to construct the FACTOR_TYPE
   * @param loss: The graduated robust loss applied to this factor
   * @param scheduler: The control param $\mu$ scheduler for this factor
   */
  GraduatedFactor(RobustLoss::shared_ptr loss,
                  GraduationScheduler::shared_ptr scheduler)
      : robustLoss_(loss), scheduler_(scheduler) {
    mu_ = std::make_shared<double>(scheduler_->muInit());
  }

  /// @brief Copy constructor
  GraduatedFactor(const GraduatedFactor& other)
      : robustLoss_(other.robustLoss_), scheduler_(other.scheduler_) {
    mu_ = std::make_shared<double>(*(other.mu_));
  }

  virtual ~GraduatedFactor() = default;

  /** @brief Linearize this factor using the convexification parameter mu
   *  @param currentEstimate: the estimate at which to linearize the factor
   */
  virtual GaussianFactor::shared_ptr linearizeGraduated(
      const Values& currentEstimate) const = 0;

  /// @brief returns the residual of the factor
  virtual double residual(const Values& currentEstimate) const = 0;

  /// @brief returns the graduated robust loss \rho_\mu(r) of the factor
  virtual double robustLoss(const Values& currentEstimate) const = 0;

  /// @brief Returns the robust loss for this graduated factor
  const RobustLoss::shared_ptr& loss() const { return robustLoss_; }

  /// @brief Returns the graduation scheduler for this factor
  const GraduationScheduler::shared_ptr& scheduler() const {
    return scheduler_;
  }

  /// @brief Copies this factor as an instance of its base type without the
  /// graduated robust loss or scheduler
  virtual NonlinearFactor::shared_ptr cloneUngraduated() const = 0;
  /// @}
};

/// @brief Instantiation of Graduated Factor wrapping any Nonlinear Factor
template <class FACTOR_TYPE>
class GenericGraduatedFactor : public FACTOR_TYPE, public GraduatedFactor {
  static_assert(std::is_base_of<NonlinearFactor, FACTOR_TYPE>::value,
                "GraduatedFactor Must be instantiated with a Factor Derived "
                "from NonlinearFactor.");

  /// @name Types
  /// @{
 public:
  typedef std::shared_ptr<GenericGraduatedFactor<FACTOR_TYPE>> shared_ptr;
  /// @}

  /// @name Factor Interface
  /// @{
 public:
  /** @brief Constructor
   * @param args: The arguments required to construct the FACTOR_TYPE
   * @param loss: The graduated robust loss applied to this factor
   * @param scheduler: The control param $\mu$ scheduler for this factor
   */
  template <class... Args>
  GenericGraduatedFactor(RobustLoss::shared_ptr loss,
                         GraduationScheduler::shared_ptr scheduler,
                         Args&&... args)
      : FACTOR_TYPE(std::forward<Args>(args)...),
        GraduatedFactor(loss, scheduler) {}

  /// @brief Copy Constructor
  /// Delegates to GraduatedFactor's copy constructor so the copy preserves the
  /// current graduation progress (mu) rather than resetting it to muInit.
  GenericGraduatedFactor(const GenericGraduatedFactor<FACTOR_TYPE>& other)
      : FACTOR_TYPE(other), GraduatedFactor(other) {}

  /// @brief Makes a deep copy of the factor
  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(
            new GenericGraduatedFactor<FACTOR_TYPE>(*this)));
  }

  /// @brief Linearizes the factor using the current value of mu
  GaussianFactor::shared_ptr linearizeGraduated(
      const Values& currentEstimate) const override {
    double residual = this->residual(currentEstimate);

    // Use base factor to linearize
    auto whitenedLinearSystem = FACTOR_TYPE::linearize(currentEstimate);
    auto [A, b] = whitenedLinearSystem->jacobian();
    const size_t outputDim = b.size();

    // Extract the non-dense linear system
    auto keys = whitenedLinearSystem->keys();
    std::vector<Matrix> Ablocks;
    size_t indexStart = 0;
    for (const auto& key : keys) {
      size_t d = currentEstimate.at(key).dim();
      Ablocks.push_back(A.block(0, indexStart, outputDim, d));
      indexStart += d;
    }

    // Weight the Linearized Blocks
    const double sqrtWeight =
        std::sqrt(robustLoss_->graduatedWeight(residual, *mu_));
    for (Matrix& Aj : Ablocks) {
      Aj *= sqrtWeight;
    }
    b *= sqrtWeight;

    // Construct a jacobian factor from the weighted system
    FastMap<Key, Matrix> AblockMap;
    for (size_t i = 0; i < Ablocks.size(); i++) {
      AblockMap[keys[i]] = Ablocks[i];
    }
    return std::make_shared<JacobianFactor>(AblockMap, b);
  }

  /// @brief Linearize the System @see NonlinearFactor::linearize
  GaussianFactor::shared_ptr linearize(
      const Values& currentEstimate) const override {
    // Delegate to linearizeGraduated which is required by the GraduatedFactor
    // Interface
    return linearizeGraduated(currentEstimate);
  }

  /// @brief Returns the graduated robust loss \rho_\mu(r)
  double error(const Values& values) const override {
    return robustLoss(values);
  }
  /// @}

  /// @name Graduated Interface
  /// @{
  /// @brief See GraduatedFactor::residual
  double residual(const Values& currentEstimate) const override {
    return std::sqrt(2.0 * FACTOR_TYPE::error(currentEstimate));
  }

  /// @brief See GraduatedFactor::robustLoss
  double robustLoss(const Values& currentEstimate) const override {
    return robustLoss_->graduatedLoss(residual(currentEstimate), *mu_);
  }

  /// @brief See GraduatedFactor::cloneUngraduated
  NonlinearFactor::shared_ptr cloneUngraduated() const override {
    return FACTOR_TYPE::clone();
  }
  /// @}
};
}  // namespace gtsam
