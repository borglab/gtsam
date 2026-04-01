/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AugmentedLagrangian.cpp
 * @brief   Augmented Lagrangian method for nonlinear constrained optimization.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#include <gtsam/constrained/AugmentedLagrangian.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/slam/AntiFactor.h>

#include <iostream>

namespace gtsam {

/** A factor that adds a constant bias term to the original factor.
 * This factor is used in augmented Lagrangian optimizer to create biased cost
 * functions.
 * Note that the noise model is stored twice (both in original factor and the
 * noisemodel of substitute factor. The noisemodel in the original factor will
 * be ignored. */
class GTSAM_EXPORT BiasedFactor : public NoiseModelFactor {
 protected:
  typedef NoiseModelFactor Base;
  typedef BiasedFactor This;

  // original factor
  Base::shared_ptr originalFactor_;
  Vector bias_;

 public:
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  BiasedFactor() {}

  /** Destructor */
  ~BiasedFactor() override {}

  /**
   * Constructor
   * @param originalFactor   original factor on X
   * @param bias  the bias term
   */
  BiasedFactor(const Base::shared_ptr& originalFactor, const Vector& bias)
      : Base(originalFactor->noiseModel(), originalFactor->keys()),
        originalFactor_(originalFactor),
        bias_(bias) {}

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(
      const Values& x,
      gtsam::OptionalMatrixVecType H = nullptr) const override {
    return originalFactor_->unwhitenedError(x, H) + bias_;
  }

  /** print */
  void print(const std::string& s, const KeyFormatter& keyFormatter =
                                       DefaultKeyFormatter) const override {
    std::cout << s << "BiasedFactor " << bias_.transpose()
              << " version of:" << std::endl;
    originalFactor_->print(s, keyFormatter);
  }

  /** Return a deep copy of this factor. */
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

};  // \class BiasedFactor

NonlinearFactorGraph AugmentedLagrangianFunction(
    const ConstrainedOptProblem& problem, const std::vector<Vector>& lambdaEq,
    const std::vector<double>& lambdaIneq, double muEq, double muIneq,
    InequalityPenaltyFunction::shared_ptr ineqConstraintPenaltyFunction,
    const double epsilon) {
  // Initialize by adding in cost factors.
  NonlinearFactorGraph graph = problem.costs();

  // Create factors corresponding to equality constraints.
  const NonlinearEqualityConstraints& eqConstraints = problem.eConstraints();
  for (size_t i = 0; i < eqConstraints.size(); i++) {
    const auto& constraint = eqConstraints.at(i);
    Vector bias = (lambdaEq[i] / muEq).cwiseProduct(constraint->sigmas());
    auto penalty_l2 = constraint->penaltyFactor(muEq);
    graph.emplace_shared<BiasedFactor>(penalty_l2, bias);
  }

  // Create factors corresponding to penalty terms of inequality constraints.
  const NonlinearInequalityConstraints& ineqConstraints =
      problem.iConstraints();
  graph.add(ineqConstraints.penaltyGraphCustom(ineqConstraintPenaltyFunction,
                                               muIneq));

  // Create factors corresponding to Lagrange multiplier terms of i-constraints.
  for (size_t i = 0; i < ineqConstraints.size(); i++) {
    const auto& constraint = ineqConstraints.at(i);
    Vector bias = constraint->sigmas() * (lambdaIneq[i] / epsilon);
    auto penalty_l2 = constraint->penaltyFactorEquality(epsilon);
    graph.emplace_shared<BiasedFactor>(penalty_l2, bias);
    graph.emplace_shared<AntiFactor>(penalty_l2);
  }

  return graph;
}

}  // namespace gtsam
