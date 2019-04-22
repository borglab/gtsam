/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ShonanAveraging.cpp
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Shonan Averaging algorithm
 */

#include <gtsam_unstable/slam/ShonanAveraging.h>

#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/KarcherMeanFactor-inl.h>
#include <gtsam_unstable/slam/FrobeniusFactor.h>

#include <iostream>
#include <map>

namespace gtsam {

static boost::mt19937 kRandomNumberGenerator(42);

/* ************************************************************************* */
ShonanAveragingParameters::ShonanAveragingParameters(const string& verbosity,
                                                     const std::string& method)
    : prior(true), karcher(true) {
  lm.setVerbosityLM(verbosity);

  // Set parameters to be similar to ceres
  LevenbergMarquardtParams::SetCeresDefaults(&lm);

  // By default, we will do conjugate gradient
  lm.linearSolverType = LevenbergMarquardtParams::Iterative;

  // Create subgraph builder parameters
  SubgraphBuilderParameters builderParameters;
  builderParameters.skeletonType = SubgraphBuilderParameters::KRUSKAL;
  builderParameters.skeletonWeight = SubgraphBuilderParameters::EQUAL;
  builderParameters.augmentationWeight = SubgraphBuilderParameters::SKELETON;
  builderParameters.augmentationFactor = 0.0;

  auto pcg = boost::make_shared<PCGSolverParameters>();

  // Choose optimization method
  if (method == "SUBGRAPH") {
    lm.iterativeParams =
        boost::make_shared<SubgraphSolverParameters>(builderParameters);
  } else if (method == "SGPC") {
    pcg->preconditioner_ =
        boost::make_shared<SubgraphPreconditionerParameters>(builderParameters);
    lm.iterativeParams = pcg;
  } else if (method == "JACOBI") {
    pcg->preconditioner_ =
        boost::make_shared<BlockJacobiPreconditionerParameters>();
    lm.iterativeParams = pcg;
  } else if (method == "QR") {
    lm.setLinearSolverType("MULTIFRONTAL_QR");
  } else if (method == "CHOLESKY") {
    lm.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
  } else {
    throw std::invalid_argument("ShonanAveragingParameters: unknown mtehod");
  }
}

/* ************************************************************************* */
ShonanAveraging::ShonanAveraging(const string& g2oFile,
                                 const ShonanAveragingParameters& parameters)
    : parameters_(parameters) {
  factors_ = parse3DFactors(g2oFile);
  poses_ = parse3DPoses(g2oFile);
}

/* ************************************************************************* */
NonlinearFactorGraph ShonanAveraging::buildGraphAt(size_t p) const {
  NonlinearFactorGraph graph;
  for (const auto& factor : factors_) {
    const auto& keys = factor->keys();
    const auto& Tij = factor->measured();
    const auto& model = factor->noiseModel();
    graph.emplace_shared<FrobeniusWormholeFactorTL>(
        keys[0], keys[1], SO3(Tij.rotation().matrix()), p, model);
  }
  return graph;
}

/* ************************************************************************* */
Values ShonanAveraging::initializeRandomlyAt(size_t p) const {
  Values initial;
  for (size_t j = 0; j < poses_.size(); j++) {
    initial.insert(j, SOn::Random(kRandomNumberGenerator, p));
  }
  return initial;
}

/* ************************************************************************* */
static Values Optimize(const NonlinearFactorGraph& graph, const Values& initial,
                       const LevenbergMarquardtParams& LMparameters) {
  LevenbergMarquardtOptimizer lm(graph, initial, LMparameters);
  Values result = lm.optimize();
  return result;
}

/* ************************************************************************* */
double ShonanAveraging::costAt(size_t p, const Values& values) const {
  const NonlinearFactorGraph graph = buildGraphAt(p);
  return graph.error(values);
}

/* ************************************************************************* */
Values ShonanAveraging::tryOptimizingAt(
    size_t p, const boost::optional<const Values&> initialEstimate) const {
  // Build graph
  auto graph = buildGraphAt(p);

  // Initialize randomly if no initial estimate is given
  // TODO(frank): add option to do chordal init
  const Values initial =
      initialEstimate ? *initialEstimate : initializeRandomlyAt(p);

  // Prior is only added here as depends on initial value (and cost is zero)
  if (parameters_.prior) {
    if (parameters_.karcher) {
      const size_t d = p * (p - 1) / 2;
      graph.emplace_shared<KarcherMeanFactor<SOn>>(graph.keys(), d);
    } else {
      graph.emplace_shared<NonlinearEquality<SOn>>(0, initial.at<SOn>(0));
    }
  }

  // Optimize
  return Optimize(graph, initial, parameters_.lm);
}

/* ************************************************************************* */
Values ShonanAveraging::projectFrom(size_t p, const Values& values) const {
  Values SO3_values;
  for (size_t j = 0; j < poses_.size(); j++) {
    const SOn Q = values.at<SOn>(j);
    assert(Q.rows() == p);
    const SO3 R = SO3::ClosestTo(Q.topLeftCorner(3, 3));
    SO3_values.insert(j, R);
  }
  return SO3_values;
}

/* ************************************************************************* */
double ShonanAveraging::cost(const Values& values) const {
  NonlinearFactorGraph graph;
  for (const auto& factor : factors_) {
    const auto& keys = factor->keys();
    const auto& Tij = factor->measured();
    const auto& model = factor->noiseModel();
    graph.emplace_shared<FrobeniusBetweenFactor<SO3>>(
        keys[0], keys[1], SO3(Tij.rotation().matrix()), model);
  }
  return graph.error(values);
}

/* ************************************************************************* */
void ShonanAveraging::run(size_t p_max) const {
  // TODO(frank): check optimality and return optimal values
  for (size_t p = 3; p <= p_max; p++) {
    tryOptimizingAt(p);
  }
}

/* ************************************************************************* */
}  // namespace gtsam
