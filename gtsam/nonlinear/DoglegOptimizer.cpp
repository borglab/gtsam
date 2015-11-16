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
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <boost/algorithm/string.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
DoglegParams::VerbosityDL DoglegParams::verbosityDLTranslator(const std::string &verbosityDL) const {
  std::string s = verbosityDL;  boost::algorithm::to_upper(s);
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
void DoglegOptimizer::iterate(void) {

  // Linearize graph
  GaussianFactorGraph::shared_ptr linear = graph_.linearize(state_.values);

  // Pull out parameters we'll use
  const bool dlVerbose = (params_.verbosityDL > DoglegParams::SILENT);

  // Do Dogleg iteration with either Multifrontal or Sequential elimination
  DoglegOptimizerImpl::IterationResult result;

  if ( params_.isMultifrontal() ) {
    GaussianBayesTree bt = *linear->eliminateMultifrontal(*params_.ordering, params_.getEliminationFunction());
    VectorValues dx_u = bt.optimizeGradientSearch();
    VectorValues dx_n = bt.optimize();
    result = DoglegOptimizerImpl::Iterate(state_.Delta, DoglegOptimizerImpl::ONE_STEP_PER_ITERATION,
      dx_u, dx_n, bt, graph_, state_.values, state_.error, dlVerbose);
  }
  else if ( params_.isSequential() ) {
    GaussianBayesNet bn = *linear->eliminateSequential(*params_.ordering, params_.getEliminationFunction());
    VectorValues dx_u = bn.optimizeGradientSearch();
    VectorValues dx_n = bn.optimize();
    result = DoglegOptimizerImpl::Iterate(state_.Delta, DoglegOptimizerImpl::ONE_STEP_PER_ITERATION,
      dx_u, dx_n, bn, graph_, state_.values, state_.error, dlVerbose);
  }
  else if ( params_.isIterative() ) {
    throw runtime_error("Dogleg is not currently compatible with the linear conjugate gradient solver");
  }
  else {
    throw runtime_error("Optimization parameter is invalid: DoglegParams::elimination");
  }

  // Maybe show output
  if(params_.verbosity >= NonlinearOptimizerParams::DELTA) result.dx_d.print("delta");

  // Create new state with new values and new error
  state_.values = state_.values.retract(result.dx_d);
  state_.error = result.f_error;
  state_.Delta = result.Delta;
  ++state_.iterations;
}

/* ************************************************************************* */
DoglegParams DoglegOptimizer::ensureHasOrdering(DoglegParams params, const NonlinearFactorGraph& graph) const {
  if (!params.ordering)
    params.ordering = Ordering::Create(params.orderingType, graph);
  return params;
}

} /* namespace gtsam */
