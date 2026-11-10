/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
#include <gtsam/constrained/AugmentedLagrangianSolver.h>
#include <gtsam/constrained/internal/GaussianAlmModel.h>

namespace gtsam {

AugmentedLagrangianSolver::AugmentedLagrangianSolver(
    const GaussianFactorGraph& costs, const GaussianFactorGraph& equalities,
    const GaussianFactorGraph& inequalities, Params::shared_ptr params)
    : costs_(costs),
      equalities_(equalities),
      inequalities_(inequalities),
      params_(params ? std::move(params) : std::make_shared<Params>()) {}

VectorValues AugmentedLagrangianSolver::optimize() const {
  return optimizeWithState().values;
}
VectorValues AugmentedLagrangianSolver::optimize(
    const VectorValues& initialValues) const {
  return optimizeWithState(initialValues).values;
}
AugmentedLagrangianSolver::State AugmentedLagrangianSolver::optimizeWithState()
    const {
  return run(nullptr);
}
AugmentedLagrangianSolver::State AugmentedLagrangianSolver::optimizeWithState(
    const VectorValues& initialValues) const {
  return run(&initialValues);
}

AugmentedLagrangianSolver::State AugmentedLagrangianSolver::run(
    const VectorValues* initialValues) const {
  if (!params_)
    throw std::invalid_argument("AugmentedLagrangianParams must not be null");
  internal::GaussianAlmModel model(costs_, equalities_, inequalities_,
                                   params_->enableProfiling);
  State initial;
  initial.values = initialValues ? *initialValues : model.zeroValues();
  model.validateValues(initial.values);
  internal::AugmentedLagrangianEngine<internal::GaussianAlmModel, State> engine(
      std::move(model), *params_);
  return engine.optimize(std::move(initial), &progress_);
}

}  // namespace gtsam
