/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GncOptimizer.cpp
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

  /// Constructor
  GncParams(const BaseOptimizerParameters& baseOptimizerParams) :
      baseOptimizerParams(baseOptimizerParams) {
  }

  /// Default constructor
  GncParams() :
      baseOptimizerParams() {
  }

  /// GNC parameters
  BaseOptimizerParameters baseOptimizerParams; /*optimization parameters used to solve the weighted least squares problem at each GNC iteration*/
  /// any other specific GNC parameters:
  RobustLossType lossType = GM; /* default loss*/
  size_t maxIterations = 100; /* maximum number of iterations*/
  double barcSq = 1.0; /* a factor is considered an inlier if factor.error() < barcSq. Note that factor.error() whitens by the covariance*/
  double muStep = 1.4; /* multiplicative factor to reduce/increase the mu in gnc */
  VerbosityGNC verbosityGNC = SILENT; /* verbosity level */
  std::vector<size_t> knownInliers = std::vector<size_t>(); /* slots in the factor graph corresponding to measurements that we know are inliers */

  /// Set the robust loss function to be used in GNC (chosen among the ones in RobustLossType)
  void setLossType(const RobustLossType type) {
    lossType = type;
  }
  /// Set the maximum number of iterations in GNC (changing the max nr of iters might lead to less accurate solutions and is not recommended)
  void setMaxIterations(const size_t maxIter) {
    std::cout
        << "setMaxIterations: changing the max nr of iters might lead to less accurate solutions and is not recommended! "
        << std::endl;
    maxIterations = maxIter;
  }
  /** Set the maximum weighted residual error for an inlier. For a factor in the form f(x) = 0.5 * || r(x) ||^2_Omega,
   * the inlier threshold is the largest value of f(x) for the corresponding measurement to be considered an inlier.
   * */
  void setInlierThreshold(const double inth) {
    barcSq = inth;
  }
  /// Set the graduated non-convexity step: at each GNC iteration, mu is updated as mu <- mu * muStep
  void setMuStep(const double step) {
    muStep = step;
  }
  /// Set the verbosity level
  void setVerbosityGNC(const VerbosityGNC verbosity) {
    verbosityGNC = verbosity;
  }
  /** (Optional) Provide a vector of measurements that must be considered inliers. The enties in the vector
   * corresponds to the slots in the factor graph. For instance, if you have a nonlinear factor graph nfg,
   * and you provide  knownIn = {0, 2, 15}, GNC will not apply outlier rejection to nfg[0], nfg[2], and nfg[15].
   * This functionality is commonly used in SLAM when one may assume the odometry is outlier free, and
   * only apply GNC to prune outliers from the loop closures
   * */
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
  /// Constructor
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

  /// Access a copy of the internal factor graph
  NonlinearFactorGraph getFactors() const {
    return NonlinearFactorGraph(nfg_);
  }
  /// Access a copy of the internal values
  Values getState() const {
    return Values(state_);
  }
  /// Access a copy of the parameters
  GncParameters getParams() const {
    return GncParameters(params_);
  }
  /// Access a copy of the GNC weights
  Vector getWeights() const {
    return weights_;
  }
  /// Compute optimal solution using graduated non-convexity
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
