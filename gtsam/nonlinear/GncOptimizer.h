/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGncOptimizer.cpp
 * @brief   Unit tests for GncOptimizer class
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

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/* ************************************************************************* */
template<class BaseOptimizerParameters>
class GncParams {
public:
  /** Verbosity levels */
  enum VerbosityGNC {
    SILENT = 0, SUMMARY, VALUES
  };

  /** Choice of robust loss function for GNC */
  enum RobustLossType {
    GM /*Geman McClure*/, TLS /*Truncated least squares*/
  };

  using BaseOptimizer = GaussNewtonOptimizer; // BaseOptimizerParameters::OptimizerType;

  GncParams(const BaseOptimizerParameters& baseOptimizerParams) :
      baseOptimizerParams(baseOptimizerParams) {
  }

  // default constructor
  GncParams() :
      baseOptimizerParams() {
  }

  BaseOptimizerParameters baseOptimizerParams;
  /// any other specific GNC parameters:
  RobustLossType lossType = GM; /* default loss*/
  size_t maxIterations = 100; /* maximum number of iterations*/
  double barcSq = 1.0; /* a factor is considered an inlier if factor.error() < barcSq. Note that factor.error() whitens by the covariance*/
  double muStep = 1.4; /* multiplicative factor to reduce/increase the mu in gnc */
  VerbosityGNC verbosityGNC = SILENT; /* verbosity level */
  std::vector<size_t> knownInliers = std::vector<size_t>(); /* slots in the factor graph corresponding to measurements that we know are inliers */

  void setLossType(const RobustLossType type) {
    lossType = type;
  }
  void setMaxIterations(const size_t maxIter) {
    std::cout
        << "setMaxIterations: changing the max nr of iters might lead to less accurate solutions and is not recommended! "
        << std::endl;
    maxIterations = maxIter;
  }
  void setInlierThreshold(const double inth) {
    barcSq = inth;
  }
  void setMuStep(const double step) {
    muStep = step;
  }
  void setVerbosityGNC(const VerbosityGNC verbosity) {
    verbosityGNC = verbosity;
  }
  void setKnownInliers(const std::vector<size_t> knownIn) {
    for (size_t i = 0; i < knownIn.size(); i++)
      knownInliers.push_back(knownIn[i]);
  }

  /// equals
  bool equals(const GncParams& other, double tol = 1e-9) const {
    return baseOptimizerParams.equals(other.baseOptimizerParams)
        && lossType == other.lossType && maxIterations == other.maxIterations
        && std::fabs(barcSq - other.barcSq) <= tol
        && std::fabs(muStep - other.muStep) <= tol
        && verbosityGNC == other.verbosityGNC
        && knownInliers == other.knownInliers;
  }

  /// print function
  void print(const std::string& str) const {
    std::cout << str << "\n";
    switch (lossType) {
    case GM:
      std::cout << "lossType: Geman McClure" << "\n";
      break;
    default:
      throw std::runtime_error("GncParams::print: unknown loss type.");
    }
    std::cout << "maxIterations: " << maxIterations << "\n";
    std::cout << "barcSq: " << barcSq << "\n";
    std::cout << "muStep: " << muStep << "\n";
    std::cout << "verbosityGNC: " << verbosityGNC << "\n";
    for (size_t i = 0; i < knownInliers.size(); i++)
      std::cout << "knownInliers: " << knownInliers[i] << "\n";
    baseOptimizerParams.print(str);
  }
};

/* ************************************************************************* */
template<class GncParameters>
class GncOptimizer {
public:
  // types etc

private:
  NonlinearFactorGraph nfg_;
  Values state_;
  GncParameters params_;
  Vector weights_; // this could be a local variable in optimize, but it is useful to make it accessible from outside

public:
  GncOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
      const GncParameters& params = GncParameters()) :
      state_(initialValues), params_(params) {

    // make sure all noiseModels are Gaussian or convert to Gaussian
    nfg_.resize(graph.size());
    for (size_t i = 0; i < graph.size(); i++) {
      if (graph[i]) {
        NoiseModelFactor::shared_ptr factor = boost::dynamic_pointer_cast<
            NoiseModelFactor>(graph[i]);
        noiseModel::Robust::shared_ptr robust = boost::dynamic_pointer_cast<
            noiseModel::Robust>(factor->noiseModel());
        if (robust) { // if the factor has a robust loss, we have to change it:
          SharedNoiseModel gaussianNoise = robust->noise();
          NoiseModelFactor::shared_ptr gaussianFactor =
              factor->cloneWithNewNoiseModel(gaussianNoise);
          nfg_[i] = gaussianFactor;
        } else { // else we directly push it back
          nfg_[i] = factor;
        }
      }
    }
  }

  /// getter functions
  NonlinearFactorGraph getFactors() const {
    return NonlinearFactorGraph(nfg_);
  }
  Values getState() const {
    return Values(state_);
  }
  GncParameters getParams() const {
    return GncParameters(params_);
  }
  Vector getWeights() const {
    return weights_;
  }

  /// implement GNC main loop, including graduating nonconvexity with mu
  Values optimize() {
    // start by assuming all measurements are inliers
    weights_ = Vector::Ones(nfg_.size());
    GaussNewtonOptimizer baseOptimizer(nfg_, state_);
    Values result = baseOptimizer.optimize();
    double mu = initializeMu();
    for (size_t iter = 0; iter < params_.maxIterations; iter++) {

      // display info
      if (params_.verbosityGNC >= GncParameters::VerbosityGNC::VALUES) {
        result.print("result\n");
        std::cout << "mu: " << mu << std::endl;
        std::cout << "weights: " << weights_ << std::endl;
      }
      // weights update
      weights_ = calculateWeights(result, mu);

      // variable/values update
      NonlinearFactorGraph graph_iter = this->makeWeightedGraph(weights_);
      GaussNewtonOptimizer baseOptimizer_iter(graph_iter, state_);
      result = baseOptimizer_iter.optimize();

      // stopping condition
      if (checkMuConvergence(mu)) {
        // display info
        if (params_.verbosityGNC >= GncParameters::VerbosityGNC::SUMMARY) {
          std::cout << "final iterations: " << iter << std::endl;
          std::cout << "final mu: " << mu << std::endl;
          std::cout << "final weights: " << weights_ << std::endl;
        }
        break;
      }

      // otherwise update mu
      mu = updateMu(mu);
    }
    return result;
  }

  /// initialize the gnc parameter mu such that loss is approximately convex (remark 5 in GNC paper)
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
    case GncParameters::GM:
      return 2 * rmax_sq / params_.barcSq; // initial mu
    default:
      throw std::runtime_error(
          "GncOptimizer::initializeMu: called with unknown loss type.");
    }
  }

  /// update the gnc parameter mu to gradually increase nonconvexity
  double updateMu(const double mu) const {
    switch (params_.lossType) {
    case GncParameters::GM:
      return std::max(1.0, mu / params_.muStep); // reduce mu, but saturate at 1
    default:
      throw std::runtime_error(
          "GncOptimizer::updateMu: called with unknown loss type.");
    }
  }

  /// check if we have reached the value of mu for which the surrogate loss matches the original loss
  bool checkMuConvergence(const double mu) const {
    switch (params_.lossType) {
    case GncParameters::GM:
      return std::fabs(mu - 1.0) < 1e-9; // mu=1 recovers the original GM function
    default:
      throw std::runtime_error(
          "GncOptimizer::checkMuConvergence: called with unknown loss type.");
    }
  }

  /// create a graph where each factor is weighted by the gnc weights
  NonlinearFactorGraph makeWeightedGraph(const Vector& weights) const {
    // make sure all noiseModels are Gaussian or convert to Gaussian
    NonlinearFactorGraph newGraph;
    newGraph.resize(nfg_.size());
    for (size_t i = 0; i < nfg_.size(); i++) {
      if (nfg_[i]) {
        NoiseModelFactor::shared_ptr factor = boost::dynamic_pointer_cast<
            NoiseModelFactor>(nfg_[i]);
        noiseModel::Gaussian::shared_ptr noiseModel =
            boost::dynamic_pointer_cast<noiseModel::Gaussian>(
                factor->noiseModel());
        if (noiseModel) {
          Matrix newInfo = weights[i] * noiseModel->information();
          SharedNoiseModel newNoiseModel = noiseModel::Gaussian::Information(
              newInfo);
          newGraph[i] = factor->cloneWithNewNoiseModel(newNoiseModel);
        } else {
          throw std::runtime_error(
              "GncOptimizer::makeWeightedGraph: unexpected non-Gaussian noise model.");
        }
      }
    }
    return newGraph;
  }

  /// calculate gnc weights
  Vector calculateWeights(const Values currentEstimate, const double mu) {
    Vector weights = Vector::Ones(nfg_.size());

    // do not update the weights that the user has decided are known inliers
    std::vector<size_t> allWeights;
    for (size_t k = 0; k < nfg_.size(); k++) {
      allWeights.push_back(k);
    }
    std::vector<size_t> unknownWeights;
    std::set_difference(allWeights.begin(), allWeights.end(),
        params_.knownInliers.begin(), params_.knownInliers.end(),
        std::inserter(unknownWeights, unknownWeights.begin()));

    // update weights of known inlier/outlier measurements
    switch (params_.lossType) {
    case GncParameters::GM: // use eq (12) in GNC paper
      for (size_t k : unknownWeights) {
        if (nfg_[k]) {
          double u2_k = nfg_[k]->error(currentEstimate); // squared (and whitened) residual
          weights[k] = std::pow(
              (mu * params_.barcSq) / (u2_k + mu * params_.barcSq), 2);
        }
      }
      return weights;
    default:
      throw std::runtime_error(
          "GncOptimizer::calculateWeights: called with unknown loss type.");
    }
  }
};

}
