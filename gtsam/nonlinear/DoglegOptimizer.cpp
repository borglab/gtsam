/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DoglegOptimizer.cpp
 * @brief
 * @author  Richard Roberts
 * @date   Feb 26, 2012
 */

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

/* ************************************************************************* */
DoglegParams::VerbosityDL DoglegParams::verbosityDLTranslator(const std::string &verbosityDL) const {
  std::string s = verbosityDL;
  // convert to upper case
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
  if (s == "SILENT") return DoglegParams::SILENT;
  if (s == "VERBOSE") return DoglegParams::VERBOSE;

  /* default is silent */
  return DoglegParams::SILENT;
}

/* ************************************************************************* */
std::string DoglegParams::verbosityDLTranslator(VerbosityDL verbosityDL) const {
  std::string s;
  switch (verbosityDL) {
  case DoglegParams::SILENT:  s = "SILENT"; break;
  case DoglegParams::VERBOSE: s = "VERBOSE"; break;
  default:                    s = "UNDEFINED"; break;
  }
  return s;
}

/* ************************************************************************* */
namespace internal {
struct DoglegState : public NonlinearOptimizerState {
  const double delta;

  DoglegState(const Values& values, double error, double delta, unsigned int iterations = 0)
      : NonlinearOptimizerState(values, error, iterations), delta(delta) {}
};
}

typedef internal::DoglegState State;

/* ************************************************************************* */
DoglegOptimizer::DoglegOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
                                 const DoglegParams& params)
    : NonlinearOptimizer(
          graph, std::unique_ptr<State>(
                     new State(initialValues, graph.error(initialValues), params.deltaInitial))),
      params_(ensureHasOrdering(params, graph)) {}

DoglegOptimizer::DoglegOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
                                 const Ordering& ordering)
    : NonlinearOptimizer(graph, std::unique_ptr<State>(
                                    new State(initialValues, graph.error(initialValues), 1.0))) {
  params_.ordering = ordering;
}

double DoglegOptimizer::getDelta() const {
  return static_cast<const State*>(state_.get())->delta;
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr DoglegOptimizer::iterate(void) {

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = graph_.linearize(state_->values);

  // Pull out parameters we'll use
  const bool dlVerbose = (params_.verbosityDL > DoglegParams::SILENT);

  // Do Dogleg iteration with either Multifrontal or Sequential elimination
  DoglegOptimizerImpl::IterationResult result;

  if ( params_.isMultifrontal() ) {
    GaussianBayesTree bt = *linear->eliminateMultifrontal(*params_.ordering, params_.getEliminationFunction());
    VectorValues dx_u = bt.optimizeGradientSearch();
    VectorValues dx_n = bt.optimize();
    result = DoglegOptimizerImpl::Iterate(getDelta(), DoglegOptimizerImpl::ONE_STEP_PER_ITERATION,
      dx_u, dx_n, bt, graph_, state_->values, state_->error, dlVerbose);
  }
  else if ( params_.isSequential() ) {
    GaussianBayesNet bn = *linear->eliminateSequential(*params_.ordering, params_.getEliminationFunction());
    VectorValues dx_u = bn.optimizeGradientSearch();
    VectorValues dx_n = bn.optimize();
    result = DoglegOptimizerImpl::Iterate(getDelta(), DoglegOptimizerImpl::ONE_STEP_PER_ITERATION,
      dx_u, dx_n, bn, graph_, state_->values, state_->error, dlVerbose);
  }
  else if ( params_.isIterative() ) {
    throw std::runtime_error("Dogleg is not currently compatible with the linear conjugate gradient solver");
  }
  else {
    throw std::runtime_error("Optimization parameter is invalid: DoglegParams::elimination");
  }

  // Maybe show output
  if(params_.verbosity >= NonlinearOptimizerParams::DELTA) result.dx_d.print("delta");

  // Create new state with new values and new error
  state_.reset(new State(state_->values.retract(result.dx_d), result.f_error, result.delta,
                         state_->iterations + 1));
  return linear;
}

/* ************************************************************************* */
DoglegParams DoglegOptimizer::ensureHasOrdering(DoglegParams params, const NonlinearFactorGraph& graph) const {
  if (!params.ordering)
    params.ordering = Ordering::Create(params.orderingType, graph);
  return params;
}

} /* namespace gtsam */
