/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GncOptimizer.h
 * @brief   The GncOptimizer class
 * @author  Jingnan Shi
 * @author  Luca Carlone
 * @author  Frank Dellaert
 *
 * Implementation of the paper: Yang, Antonante, Tzoumas, Carlone, "Graduated Non-Convexity for Robust Spatial Perception:
 * From Non-Minimal Solvers to Global Outlier Rejection", ICRA/RAL, 2020. (arxiv version: https://arxiv.org/pdf/1909.08605.pdf)
 *
 * See also:
 * Antonante, Tzoumas, Yang, Carlone, "Outlier-Robust Estimation: Hardness, Minimally-Tuned Algorithms, and Applications",
 * arxiv: https://arxiv.org/pdf/2007.15109.pdf, 2020.
 */

#pragma once

#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/* ************************************************************************* */
template<class GncParameters>
class GncOptimizer {
 public:
  /// For each parameter, specify the corresponding optimizer: e.g., GaussNewtonParams -> GaussNewtonOptimizer.
  typedef typename GncParameters::OptimizerType BaseOptimizer;

 private:
  NonlinearFactorGraph nfg_; ///< Original factor graph to be solved by GNC.
  Values state_; ///< Initial values to be used at each iteration by GNC.
  GncParameters params_; ///< GNC parameters.
  Vector weights_;  ///< Weights associated to each factor in GNC (this could be a local variable in optimize, but it is useful to make it accessible from outside).

 public:
  /// Constructor.
  GncOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
               const GncParameters& params = GncParameters())
      : state_(initialValues),
        params_(params) {

    // make sure all noiseModels are Gaussian or convert to Gaussian
    nfg_.resize(graph.size());
    for (size_t i = 0; i < graph.size(); i++) {
      if (graph[i]) {
        NoiseModelFactor::shared_ptr factor = boost::dynamic_pointer_cast<
            NoiseModelFactor>(graph[i]);
        auto robust = boost::dynamic_pointer_cast<
            noiseModel::Robust>(factor->noiseModel());
        // if the factor has a robust loss, we remove the robust loss
        nfg_[i] = robust ? factor-> cloneWithNewNoiseModel(robust->noise()) : factor;
      }
    }
  }

  /// Access a copy of the internal factor graph.
  const NonlinearFactorGraph& getFactors() const { return nfg_; }

  /// Access a copy of the internal values.
  const Values& getState() const { return state_; }

  /// Access a copy of the parameters.
  const GncParameters& getParams() const { return params_;}

  /// Access a copy of the GNC weights.
  const Vector& getWeights() const { return weights_;}

  /// Compute optimal solution using graduated non-convexity.
  Values optimize() {
    // start by assuming all measurements are inliers
    weights_ = Vector::Ones(nfg_.size());
    BaseOptimizer baseOptimizer(nfg_, state_);
    Values result = baseOptimizer.optimize();
    double mu = initializeMu();
    double prev_cost = nfg_.error(result);
    double cost = 0.0;  // this will be updated in the main loop

    // handle the degenerate case that corresponds to small
    // maximum residual errors at initialization
    // For GM: if residual error is small, mu -> 0
    // For TLS: if residual error is small, mu -> -1
    if (mu <= 0) {
      if (params_.verbosity >= GncParameters::Verbosity::SUMMARY) {
        std::cout << "GNC Optimizer stopped because maximum residual at "
                  "initialization is small."
                  << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::VALUES) {
        result.print("result\n");
        std::cout << "mu: " << mu << std::endl;
      }
      return result;
    }

    size_t iter;
    for (iter = 0; iter < params_.maxIterations; iter++) {

      // display info
      if (params_.verbosity >= GncParameters::Verbosity::VALUES) {
        std::cout << "iter: " << iter << std::endl;
        result.print("result\n");
        std::cout << "mu: " << mu << std::endl;
        std::cout << "weights: " << weights_ << std::endl;
      }
      // weights update
      weights_ = calculateWeights(result, mu);

      // variable/values update
      NonlinearFactorGraph graph_iter = this->makeWeightedGraph(weights_);
      BaseOptimizer baseOptimizer_iter(graph_iter, state_);
      result = baseOptimizer_iter.optimize();

      // stopping condition
      cost = graph_iter.error(result);
      if (checkConvergence(mu, weights_, cost, prev_cost)) {
        break;
      }

      // update mu
      mu = updateMu(mu);

      // get ready for next iteration
      prev_cost = cost;

      // display info
      if (params_.verbosity >= GncParameters::Verbosity::VALUES) {
        std::cout << "previous cost: " << prev_cost << std::endl;
        std::cout << "current cost: " << cost << std::endl;
      }
    }
    // display info
    if (params_.verbosity >= GncParameters::Verbosity::SUMMARY) {
      std::cout << "final iterations: " << iter << std::endl;
      std::cout << "final mu: " << mu << std::endl;
      std::cout << "final weights: " << weights_ << std::endl;
      std::cout << "previous cost: " << prev_cost << std::endl;
      std::cout << "current cost: " << cost << std::endl;
    }
    return result;
  }

  /// Initialize the gnc parameter mu such that loss is approximately convex (remark 5 in GNC paper).
  double initializeMu() const {
    // compute largest error across all factors
    double rmax_sq = 0.0;
    for (size_t i = 0; i < nfg_.size(); i++) {
      if (nfg_[i]) {
        rmax_sq = std::max(rmax_sq, nfg_[i]->error(state_));
      }
    }
    // set initial mu
    switch (params_.lossType) {
      case GncLossType::GM:
        // surrogate cost is convex for large mu
        return 2 * rmax_sq / params_.barcSq;  // initial mu
      case GncLossType::TLS:
        /* initialize mu to the value specified in Remark 5 in GNC paper.
         surrogate cost is convex for mu close to zero
         degenerate case: 2 * rmax_sq - params_.barcSq < 0 (handled in the main loop)
         according to remark mu = params_.barcSq / (2 * rmax_sq - params_.barcSq) = params_.barcSq/ excessResidual
         however, if the denominator is 0 or negative, we return mu = -1 which leads to termination of the main GNC loop
         */
        return
            (2 * rmax_sq - params_.barcSq) > 0 ?
                params_.barcSq / (2 * rmax_sq - params_.barcSq) : -1;
      default:
        throw std::runtime_error(
            "GncOptimizer::initializeMu: called with unknown loss type.");
    }
  }

  /// Update the gnc parameter mu to gradually increase nonconvexity.
  double updateMu(const double mu) const {
    switch (params_.lossType) {
      case GncLossType::GM:
        // reduce mu, but saturate at 1 (original cost is recovered for mu -> 1)
        return std::max(1.0, mu / params_.muStep);
      case GncLossType::TLS:
        // increases mu at each iteration (original cost is recovered for mu -> inf)
        return mu * params_.muStep;
      default:
        throw std::runtime_error(
            "GncOptimizer::updateMu: called with unknown loss type.");
    }
  }

  /// Check if we have reached the value of mu for which the surrogate loss matches the original loss.
  bool checkMuConvergence(const double mu) const {
    bool muConverged = false;
    switch (params_.lossType) {
      case GncLossType::GM:
        muConverged = std::fabs(mu - 1.0) < 1e-9;  // mu=1 recovers the original GM function
        break;
      case GncLossType::TLS:
        muConverged = false;  // for TLS there is no stopping condition on mu (it must tend to infinity)
        break;
      default:
        throw std::runtime_error(
            "GncOptimizer::checkMuConvergence: called with unknown loss type.");
    }
    if (muConverged && params_.verbosity >= GncParameters::Verbosity::SUMMARY)
      std::cout << "muConverged = true " << std::endl;
    return muConverged;
  }

  /// Check convergence of relative cost differences.
  bool checkCostConvergence(const double cost, const double prev_cost) const {
    bool costConverged = std::fabs(cost - prev_cost) / std::max(prev_cost, 1e-7)
        < params_.relativeCostTol;
    if (costConverged && params_.verbosity >= GncParameters::Verbosity::SUMMARY)
      std::cout << "checkCostConvergence = true " << std::endl;
    return costConverged;
  }

  /// Check convergence of weights to binary values.
  bool checkWeightsConvergence(const Vector& weights) const {
    bool weightsConverged = false;
    switch (params_.lossType) {
      case GncLossType::GM:
        weightsConverged = false;  // for GM, there is no clear binary convergence for the weights
        break;
      case GncLossType::TLS:
        weightsConverged = true;
        for (size_t i = 0; i < weights.size(); i++) {
          if (std::fabs(weights[i] - std::round(weights[i]))
              > params_.weightsTol) {
            weightsConverged = false;
            break;
          }
        }
        break;
      default:
        throw std::runtime_error(
            "GncOptimizer::checkWeightsConvergence: called with unknown loss type.");
    }
    if (weightsConverged
        && params_.verbosity >= GncParameters::Verbosity::SUMMARY)
      std::cout << "weightsConverged = true " << std::endl;
    return weightsConverged;
  }

  /// Check for convergence between consecutive GNC iterations.
  bool checkConvergence(const double mu, const Vector& weights,
                        const double cost, const double prev_cost) const {
    return checkCostConvergence(cost, prev_cost)
        || checkWeightsConvergence(weights) || checkMuConvergence(mu);
  }

  /// Create a graph where each factor is weighted by the gnc weights.
  NonlinearFactorGraph makeWeightedGraph(const Vector& weights) const {
    // make sure all noiseModels are Gaussian or convert to Gaussian
    NonlinearFactorGraph newGraph;
    newGraph.resize(nfg_.size());
    for (size_t i = 0; i < nfg_.size(); i++) {
      if (nfg_[i]) {
        auto factor = boost::dynamic_pointer_cast<
            NoiseModelFactor>(nfg_[i]);
        auto noiseModel =
            boost::dynamic_pointer_cast<noiseModel::Gaussian>(
                factor->noiseModel());
        if (noiseModel) {
          Matrix newInfo = weights[i] * noiseModel->information();
          auto newNoiseModel = noiseModel::Gaussian::Information(newInfo);
          newGraph[i] = factor->cloneWithNewNoiseModel(newNoiseModel);
        } else {
          throw std::runtime_error(
              "GncOptimizer::makeWeightedGraph: unexpected non-Gaussian noise model.");
        }
      }
    }
    return newGraph;
  }

  /// Calculate gnc weights.
  Vector calculateWeights(const Values& currentEstimate, const double mu) {
    Vector weights = Vector::Ones(nfg_.size());

    // do not update the weights that the user has decided are known inliers
    std::vector<size_t> allWeights;
    for (size_t k = 0; k < nfg_.size(); k++) {
      allWeights.push_back(k);
    }
    std::vector<size_t> unknownWeights;
    std::set_difference(allWeights.begin(), allWeights.end(),
                        params_.knownInliers.begin(),
                        params_.knownInliers.end(),
                        std::inserter(unknownWeights, unknownWeights.begin()));

    // update weights of known inlier/outlier measurements
    switch (params_.lossType) {
      case GncLossType::GM: {  // use eq (12) in GNC paper
        for (size_t k : unknownWeights) {
          if (nfg_[k]) {
            double u2_k = nfg_[k]->error(currentEstimate);  // squared (and whitened) residual
            weights[k] = std::pow(
                (mu * params_.barcSq) / (u2_k + mu * params_.barcSq), 2);
          }
        }
        return weights;
      }
      case GncLossType::TLS: {  // use eq (14) in GNC paper
        double upperbound = (mu + 1) / mu * params_.barcSq;
        double lowerbound = mu / (mu + 1) * params_.barcSq;
        for (size_t k : unknownWeights) {
          if (nfg_[k]) {
            double u2_k = nfg_[k]->error(currentEstimate);  // squared (and whitened) residual
            if (u2_k >= upperbound) {
              weights[k] = 0;
            } else if (u2_k <= lowerbound) {
              weights[k] = 1;
            } else {
              weights[k] = std::sqrt(params_.barcSq * mu * (mu + 1) / u2_k)
                  - mu;
            }
          }
        }
        return weights;
      }
      default:
        throw std::runtime_error(
            "GncOptimizer::calculateWeights: called with unknown loss type.");
    }
  }
};

}
