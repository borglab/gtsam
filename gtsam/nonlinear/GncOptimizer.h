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

#include <algorithm>

#include <gtsam/linear/LossFunctions.h>
#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {
/*
 * Quantile of chi-squared distribution with given degrees of freedom at probability alpha.
 * Equivalent to chi2inv in Matlab.
 */
GTSAM_EXPORT double Chi2inv(const double alpha, const size_t dofs);

/**
 * @enum GncFactorType
 * @brief Enum to classify factor types in GNC optimization.
 */
enum class GncFactorType {
  Normal,         ///< Normal case.
  Inlier,         ///< Factor is a known inlier.
  Outlier,        ///< Factor is a known outlier.
  NonNoiseModel,  ///< Factor does not have a noise model
  NullPointer     ///< Factor pointer is null.
};

GTSAM_EXPORT bool isNullType(GncFactorType type);

GTSAM_EXPORT bool isNonNoiseModelType(GncFactorType type);

GTSAM_EXPORT bool needsWeightUpdate(GncFactorType type);

GTSAM_EXPORT bool hasNoise(GncFactorType type);

/* ************************************************************************* */
template<class GncParameters>
class GncOptimizer {
 public:
  /// For each parameter, specify the corresponding optimizer: e.g., GaussNewtonParams -> GaussNewtonOptimizer.
  typedef typename GncParameters::OptimizerType BaseOptimizer;

 private:
  /// Original factor graph to be solved by GNC.
  NonlinearFactorGraph nfg_;

  /// Initial values to be used at each iteration by GNC.
  Values state_;

  /// GNC parameters.
  GncParameters params_;

  /// Weights associated to each factor in GNC (accessible from outside).
  Vector weights_;

  /// Inlier thresholds. A factor is considered an inlier if factor.error() <
  /// barcSq_[i]. Note: factor.error() whitens by the covariance.
  Vector barcSq_;

  /// Cached factor types for GNC.
  std::vector<GncFactorType> factorTypes_;

 public:
  /// Constructor.
  GncOptimizer(const NonlinearFactorGraph& graph, const Values& initialValues,
               const GncParameters& params = GncParameters())
      : state_(initialValues),
        params_(params) {

    // make sure all noiseModels are Gaussian or convert to Gaussian
    nfg_.resize(graph.size());
    factorTypes_.assign(graph.size(), GncFactorType::NullPointer);
    for (size_t i = 0; i < graph.size(); i++) {
      if (!graph[i]) {
        factorTypes_[i] = GncFactorType::NullPointer;
        continue;
      }
      NoiseModelFactor::shared_ptr factor = graph.at<NoiseModelFactor>(i);
      if (!factor) {
        if (!params.allowNonNoiseModelFactors) {
          throw std::runtime_error("GncOptimizer::constructor: the user must set allowNonNoiseModelFactors as"
            " true if the factor graph contains factors without noise model.");
        }
        nfg_[i] = graph[i];
        factorTypes_[i] = GncFactorType::NonNoiseModel;
        continue;
      }
      auto robust =
            std::dynamic_pointer_cast<noiseModel::Robust>(factor->noiseModel());
      // if the factor has a robust loss, we remove the robust loss
      nfg_[i] = robust ? factor-> cloneWithNewNoiseModel(robust->noise()) : factor;
      factorTypes_[i] = GncFactorType::Normal;
    }

    // check that known inliers are in the graph
    for (size_t i = 0; i < params.knownInliers.size(); i++){
      if( params.knownInliers[i] > nfg_.size()-1 || isNullType(factorTypes_[params.knownInliers[i]])) { // outside graph
        throw std::runtime_error("GncOptimizer::constructor: the user has selected one or more measurements"
                  "that are not in the factor graph to be known inliers.");
      }
      if (!isNonNoiseModelType(factorTypes_[params.knownInliers[i]])) {
        factorTypes_[params.knownInliers[i]] = GncFactorType::Inlier;
      }
    }
    // check that known outliers are in the graph
    for (size_t i = 0; i < params.knownOutliers.size(); i++){
      if( params.knownOutliers[i] > nfg_.size()-1 || isNullType(factorTypes_[params.knownOutliers[i]])) { // outside graph
        throw std::runtime_error("GncOptimizer::constructor: the user has selected one or more measurements"
                  "that are not in the factor graph to be known outliers.");
      }
      if (!needsWeightUpdate(factorTypes_[params.knownOutliers[i]])) {
        // it can only be Normal, Inlier, or NonNoiseModel here so this works
        throw std::runtime_error("GncOptimizer::constructor: the user has selected one or more measurements"
                  " to be an outlier that is either an inlier or a non noise model factor.");
      }
      factorTypes_[params.knownOutliers[i]] = GncFactorType::Outlier;
    }

    // initialize weights (if we don't have prior knowledge of inliers/outliers
    // the weights are all initialized to 1.
    weights_ = initializeWeightsFromKnownInliersAndOutliers();

    // set default barcSq_ (inlier threshold)
    double alpha = 0.99; // with this (default) probability, inlier residuals are smaller than barcSq_
    setInlierCostThresholdsAtProbability(alpha);
  }

  /** Set the maximum weighted residual error for an inlier (same for all factors). For a factor in the form f(x) = 0.5 * || r(x) ||^2_Omega,
   * the inlier threshold is the largest value of f(x) for the corresponding measurement to be considered an inlier.
   * In other words, an inlier at x is such that 0.5 * || r(x) ||^2_Omega <= barcSq.
   * Assuming an isotropic measurement covariance sigma^2 * Identity, the cost becomes: 0.5 * 1/sigma^2 || r(x) ||^2 <= barcSq.
   * Hence || r(x) ||^2 <= 2 * barcSq * sigma^2.
   * */
  void setInlierCostThresholds(const double inth) {
    barcSq_ = inth * Vector::Ones(nfg_.size());
  }

  /** Set the maximum weighted residual error for an inlier (one for each factor). For a factor in the form f(x) = 0.5 * || r(x) ||^2_Omega,
   * the inlier threshold is the largest value of f(x) for the corresponding measurement to be considered an inlier.
   * In other words, an inlier at x is such that 0.5 * || r(x) ||^2_Omega <= barcSq.
   * */
  void setInlierCostThresholds(const Vector& inthVec) {
    barcSq_ = inthVec;
  }

  /** Set the maximum weighted residual error threshold by specifying the probability
   * alpha that the inlier residuals are smaller than that threshold
   * */
  void setInlierCostThresholdsAtProbability(const double alpha) {
    barcSq_  = Vector::Ones(nfg_.size()); // initialize
    for (size_t k = 0; k < nfg_.size(); k++) {
      if (hasNoise(factorTypes_[k])) {
        barcSq_[k] = 0.5 * Chi2inv(alpha, nfg_[k]->dim()); // 0.5 derives from the error definition in gtsam
      }
    }
  }

  /** Set weights for each factor. This is typically not needed, but
   * provides an extra interface for the user to initialize the weights
   * */
  void setWeights(const Vector w) {
    if (size_t(w.size()) != nfg_.size()) {
      throw std::runtime_error(
          "GncOptimizer::setWeights: the number of specified weights"
          " does not match the size of the factor graph.");
    }
    weights_ = w;
  }

  /// Access a copy of the internal factor graph.
  const NonlinearFactorGraph& getFactors() const { return nfg_; }

  /// Access a copy of the internal values.
  const Values& getState() const { return state_; }

  /// Access a copy of the parameters.
  const GncParameters& getParams() const { return params_;}

  /// Access a copy of the GNC weights.
  const Vector& getWeights() const { return weights_;}

  /// Get the inlier threshold.
  const Vector& getInlierCostThresholds() const {return barcSq_;}

  /// Equals.
  bool equals(const GncOptimizer& other, double tol = 1e-9) const {
    return nfg_.equals(other.getFactors())
        && equal(weights_, other.getWeights())
        && params_.equals(other.getParams())
        && equal(barcSq_, other.getInlierCostThresholds());
  }

  Vector initializeWeightsFromKnownInliersAndOutliers() const{
    Vector weights = Vector::Ones(nfg_.size());
    // we do not loop through the factorTypes_ vector because in general params_.knownOutliers will always be smaller
    for (size_t i = 0; i < params_.knownOutliers.size(); i++){
      weights[ params_.knownOutliers[i] ] = 0.0; // known to be outliers
    }
    return weights;
  }

  /// Compute optimal solution using graduated non-convexity.
  Values optimize() {
    validateLossSchedulerCombination();
    NonlinearFactorGraph graph_initial = this->makeWeightedGraph(weights_);
    BaseOptimizer baseOptimizer(
        graph_initial, state_, params_.baseOptimizerParams);
    Values result = baseOptimizer.optimize();
    double mu = initializeMu();
    double prev_cost = graph_initial.error(result);
    double cost = 0.0;  // this will be updated in the main loop

    // handle the degenerate case that corresponds to small
    // maximum residual errors at initialization
    // For GM: if residual error is small, mu -> 0
    // For TLS: if residual error is small, mu -> -1
    int nrUnknownInOrOut = 0;
    for (GncFactorType t : factorTypes_) {
      if (needsWeightUpdate(t)) {
        nrUnknownInOrOut++;
      }
    }
    // ^^ number of measurements that are not known to be inliers or outliers (GNC will need to figure them out)
    if (mu <= 0 || nrUnknownInOrOut == 0) { // no need to even call GNC in this case
      if (mu <= 0 && params_.verbosity >= GncParameters::Verbosity::SUMMARY) {
        std::cout << "GNC Optimizer stopped because maximum residual at "
                  "initialization is small."
                  << std::endl;
      }
      if (nrUnknownInOrOut==0 && params_.verbosity >= GncParameters::Verbosity::SUMMARY) {
        std::cout << "GNC Optimizer stopped because all measurements are already known to be inliers or outliers"
                  << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::MU) {
        std::cout << "mu: " << mu << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::VALUES) {
        result.print("result\n");
      }
      return result;
    }

    size_t iter;
    for (iter = 0; iter < params_.maxIterations; iter++) {

      // display info
      if (params_.verbosity >= GncParameters::Verbosity::MU) {
        std::cout << "iter: " << iter << std::endl;
        std::cout << "mu: " << mu << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::WEIGHTS) {
        std::cout << "weights: " << weights_ << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::VALUES) {
        result.print("result\n");
      }
      // weights update
      weights_ = calculateWeights(result, mu);

      // variable/values update
      NonlinearFactorGraph graph_iter = this->makeWeightedGraph(weights_);
      BaseOptimizer baseOptimizer_iter(
          graph_iter, state_, params_.baseOptimizerParams);
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
      std::cout << "previous cost: " << prev_cost << std::endl;
      std::cout << "current cost: " << cost << std::endl;
    }
    if (params_.verbosity >= GncParameters::Verbosity::WEIGHTS) {
      std::cout << "final weights: " << weights_ << std::endl;
    }
    return result;
  }

  void validateLossSchedulerCombination() const {
    if (params_.lossType == GncLossType::GM &&
        params_.scheduler != GncScheduler::Linear) {
      throw std::runtime_error(
          "GncOptimizer::optimize: scheduler must be Linear for GM.");
    }
    if (params_.lossType == GncLossType::TLS) {
      // Linear and SuperLinear are both valid for TLS.
      return;
    }
  }

  /// Initialize the gnc parameter mu such that loss is approximately convex (remark 5 in GNC paper).
  double initializeMu() const {

    double mu_init = 0.0;
    // initialize mu to the value specified in Remark 5 in GNC paper.
    switch (params_.lossType) {
      case GncLossType::GM:
        /* surrogate cost is convex for large mu. initialize as in remark 5 in GNC paper.
         Since barcSq_ can be different for each factor, we compute the max of the quantity in remark 5 in GNC paper
         */
        for (size_t k = 0; k < nfg_.size(); k++) {
          if (hasNoise(factorTypes_[k])) {
            mu_init = std::max(mu_init, 2 * nfg_[k]->error(state_) / barcSq_[k]);
          }
        }
        return mu_init;  // initial mu
      case GncLossType::TLS:
        /* surrogate cost is convex for mu close to zero. initialize as in remark 5 in GNC paper.
         degenerate case: 2 * rmax_sq - params_.barcSq < 0 (handled in the main loop)
         according to remark mu = params_.barcSq / (2 * rmax_sq - params_.barcSq) = params_.barcSq/ excessResidual
         however, if the denominator is 0 or negative, we return mu = -1 which leads to termination of the main GNC loop.
         Since barcSq_ can be different for each factor, we look for the minimimum (positive) quantity in remark 5 in GNC paper
         */
        mu_init = std::numeric_limits<double>::infinity();
        for (size_t k = 0; k < nfg_.size(); k++) {
          if (hasNoise(factorTypes_[k])) {
            double rk = nfg_[k]->error(state_);
            mu_init = (2 * rk - barcSq_[k]) > 0 ? // if positive, update mu, otherwise keep same
                std::min(mu_init, barcSq_[k] / (2 * rk - barcSq_[k]) ) : mu_init;
          }
        }
        if (mu_init >= 0 && mu_init < 1e-6){
          mu_init = 1e-6; // if mu ~ 0 (but positive), that means we have measurements with large errors,
          // i.e., rk > barcSq_[k] and rk very large, hence we threshold to 1e-6 to avoid mu = 0
        }
  
        return mu_init > 0 && !std::isinf(mu_init) ? mu_init : -1; // if mu <= 0 or mu = inf, return -1,
        // which leads to termination of the main gnc loop. In this case, all residuals are already below the threshold
        // and there is no need to robustify (TLS = least squares)
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
        switch (params_.scheduler) {
          case GncScheduler::SuperLinear: {
            if (mu < 1) return std::min(std::sqrt(mu) * params_.muStep, params_.muMax);
            return std::min(mu * params_.muStep, params_.muMax);
          }
          case GncScheduler::Linear: {
            return mu * params_.muStep;
          }
          default:
            throw std::runtime_error("GncOptimizer::updateMu: unknown scheduler type.");
        }
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
    if (costConverged && params_.verbosity >= GncParameters::Verbosity::SUMMARY){
      std::cout << "checkCostConvergence = true (prev. cost = " << prev_cost
                << ", curr. cost = " << cost << ")" << std::endl;
    }
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
        for (int i = 0; i < weights.size(); i++) {
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
      if (!isNullType(factorTypes_[i])) {
        if (!hasNoise(factorTypes_[i])) {
          // Keep non NoiseModel factors same.
          newGraph[i] = nfg_[i];
          continue;
        }
        auto factor = std::static_pointer_cast<NoiseModelFactor>(nfg_[i]);
        auto noiseModel = std::dynamic_pointer_cast<noiseModel::Gaussian>(
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
    Vector weights = initializeWeightsFromKnownInliersAndOutliers();

    // update weights of unknown measurements
    switch (params_.lossType) {
      case GncLossType::GM: {  // use eq (12) in GNC paper
        for (size_t k = 0; k < nfg_.size(); k++) {
          if (needsWeightUpdate(factorTypes_[k])) {
            double u2_k = nfg_[k]->error(currentEstimate);  // squared (and whitened) residual
            weights[k] = noiseModel::mEstimator::GemanMcClure::Weight(u2_k, mu * barcSq_[k]);
          }
        }
        return weights;
      }
      case GncLossType::TLS: {
        for (size_t k = 0; k < nfg_.size(); k++) {
          if (needsWeightUpdate(factorTypes_[k])) {
            double u2_k = nfg_[k]->error(currentEstimate);  // squared (and whitened) residual
            switch (params_.scheduler) {
              case GncScheduler::SuperLinear: {
                double lowerbound = barcSq_[k];
                double upperbound = ((mu + 1.0) * (mu + 1.0) / (mu * mu)) * barcSq_[k];
                auto w = noiseModel::mEstimator::TruncatedLeastSquares::Weight(u2_k, lowerbound, upperbound);
                if (w) {
                  weights[k] = *w;
                }
                else {
                  double transition_weight = std::sqrt(barcSq_[k] / u2_k) * (mu + 1.0)  - mu;
                  weights[k] = std::clamp(transition_weight, 0.0, 1.0);
                }
                break;
              }
              case GncScheduler::Linear: {  // use eq (14) in GNC paper
                double upperbound = ((mu + 1.0) / mu) * barcSq_[k];
                double lowerbound = (mu / (mu + 1.0)) * barcSq_[k];
                auto w = noiseModel::mEstimator::TruncatedLeastSquares::Weight(u2_k, lowerbound, upperbound);
                if (w) {
                  weights[k] = *w;
                }
                else {
                  double transition_weight = std::sqrt(barcSq_[k] * mu * (mu + 1.0) / u2_k) - mu;
                  weights[k] = std::clamp(transition_weight, 0.0, 1.0);
                }
                break;
              }
              default:
                throw std::runtime_error("GncOptimizer::calculateWeights: unknown scheduler type.");
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
