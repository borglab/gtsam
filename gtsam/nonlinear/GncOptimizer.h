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
#include <chrono>

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

/// Timing of one GNC outer iteration (all in seconds).
struct GncIterationTiming {
  double weightsUpdateElapsed = 0.0;   ///< calculateWeights (per-factor errors)
  double makeGraphElapsed = 0.0;       ///< makeWeightedGraph (factor cloning)
  double baseOptimizeElapsed = 0.0;    ///< inner optimizer construction + optimize()
  double costEvaluationElapsed = 0.0;  ///< weighted graph error for convergence check
  double totalElapsed = 0.0;
};

/// Timing of a full GncOptimizer::optimize() call.
struct GncTiming {
  double initialOptimizeElapsed = 0.0;  ///< optimize before the GNC loop
  double totalElapsed = 0.0;
  std::vector<GncIterationTiming> iterations;

  double sumWeightsUpdate() const {
    double sum = 0.0;
    for (const auto& it : iterations) sum += it.weightsUpdateElapsed;
    return sum;
  }
  double sumMakeGraph() const {
    double sum = 0.0;
    for (const auto& it : iterations) sum += it.makeGraphElapsed;
    return sum;
  }
  double sumBaseOptimize() const {
    double sum = 0.0;
    for (const auto& it : iterations) sum += it.baseOptimizeElapsed;
    return sum;
  }
  double sumCostEvaluation() const {
    double sum = 0.0;
    for (const auto& it : iterations) sum += it.costEvaluationElapsed;
    return sum;
  }
};

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

  /// Timing of the last optimize() call.
  GncTiming timing_;

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

  /// Get the timing of the last optimize() call.
  const GncTiming& getTiming() const { return timing_; }

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
    using Clock = std::chrono::steady_clock;
    const auto elapsedSince = [](Clock::time_point start) {
      return std::chrono::duration<double>(Clock::now() - start).count();
    };
    timing_ = GncTiming();
    const auto totalStart = Clock::now();

    validateLossSchedulerCombination();
    NonlinearFactorGraph graph_initial = this->makeWeightedGraph(weights_);
    BaseOptimizer baseOptimizer(
        graph_initial, state_, params_.baseOptimizerParams);
    Values result = baseOptimizer.optimize();
    timing_.initialOptimizeElapsed = elapsedSince(totalStart);
    double lambda = initializeLambda();
    double prev_cost = graph_initial.error(result);
    double cost = 0.0;  // this will be updated in the main loop

    // handle the degenerate case that corresponds to small
    // maximum residual errors at initialization
    // For GM: if residual error is small, lambda -> 0
    // For TLS: if residual error is small, lambda -> -1
    int nrUnknownInOrOut = 0;
    for (GncFactorType t : factorTypes_) {
      if (needsWeightUpdate(t)) {
        nrUnknownInOrOut++;
      }
    }
    // ^^ number of measurements that are not known to be inliers or outliers (GNC will need to figure them out)
    if (lambda <= 0 || nrUnknownInOrOut == 0) { // no need to even call GNC in this case
      if (lambda <= 0 && params_.verbosity >= GncParameters::Verbosity::SUMMARY) {
        std::cout << "GNC Optimizer stopped because maximum residual at "
                  "initialization is small."
                  << std::endl;
      }
      if (nrUnknownInOrOut==0 && params_.verbosity >= GncParameters::Verbosity::SUMMARY) {
        std::cout << "GNC Optimizer stopped because all measurements are already known to be inliers or outliers"
                  << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::LAMBDA) {
        std::cout << "lambda: " << lambda << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::VALUES) {
        result.print("result\n");
      }
      timing_.totalElapsed = elapsedSince(totalStart);
      return result;
    }

    size_t iter;
    for (iter = 0; iter < params_.maxIterations; iter++) {
      const auto iterationStart = Clock::now();
      GncIterationTiming iterationTiming;

      // display info
      if (params_.verbosity >= GncParameters::Verbosity::LAMBDA) {
        std::cout << "iter: " << iter << std::endl;
        std::cout << "lambda: " << lambda << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::WEIGHTS) {
        std::cout << "weights: " << weights_ << std::endl;
      }
      if (params_.verbosity >= GncParameters::Verbosity::VALUES) {
        result.print("result\n");
      }
      // weights update
      auto stageStart = Clock::now();
      weights_ = calculateWeights(result, lambda);
      iterationTiming.weightsUpdateElapsed = elapsedSince(stageStart);

      // variable/values update
      stageStart = Clock::now();
      NonlinearFactorGraph graph_iter = this->makeWeightedGraph(weights_);
      iterationTiming.makeGraphElapsed = elapsedSince(stageStart);
      stageStart = Clock::now();
      BaseOptimizer baseOptimizer_iter(
          graph_iter, state_, params_.baseOptimizerParams);
      result = baseOptimizer_iter.optimize();
      iterationTiming.baseOptimizeElapsed = elapsedSince(stageStart);

      // stopping condition
      stageStart = Clock::now();
      cost = graph_iter.error(result);
      iterationTiming.costEvaluationElapsed = elapsedSince(stageStart);
      iterationTiming.totalElapsed = elapsedSince(iterationStart);
      timing_.iterations.push_back(iterationTiming);
      if (checkConvergence(lambda, weights_, cost, prev_cost)) {
        break;
      }

      // update lambda
      lambda = updateLambda(lambda);

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
      std::cout << "final lambda: " << lambda << std::endl;
      std::cout << "previous cost: " << prev_cost << std::endl;
      std::cout << "current cost: " << cost << std::endl;
    }
    if (params_.verbosity >= GncParameters::Verbosity::WEIGHTS) {
      std::cout << "final weights: " << weights_ << std::endl;
    }
    timing_.totalElapsed = elapsedSince(totalStart);
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

  /// Initialize the gnc parameter lambda such that loss is approximately convex (remark 5 in GNC paper).
  double initializeLambda() const {

    double lambdaInit = 0.0;
    // initialize lambda to the value specified in Remark 5 in GNC paper.
    switch (params_.lossType) {
      case GncLossType::GM:
        /* surrogate cost is convex for large lambda. initialize as in remark 5 in GNC paper.
         Since barcSq_ can be different for each factor, we compute the max of the quantity in remark 5 in GNC paper
         */
        for (size_t k = 0; k < nfg_.size(); k++) {
          if (hasNoise(factorTypes_[k])) {
            lambdaInit = std::max(lambdaInit, 2 * nfg_[k]->error(state_) / barcSq_[k]);
          }
        }
        return lambdaInit;  // initial lambda
      case GncLossType::TLS:
        /* surrogate cost is convex for lambda close to zero. initialize as in remark 5 in GNC paper.
         degenerate case: 2 * rmax_sq - params_.barcSq < 0 (handled in the main loop)
         according to remark lambda = params_.barcSq / (2 * rmax_sq - params_.barcSq) = params_.barcSq/ excessResidual
         however, if the denominator is 0 or negative, we return lambda = -1 which leads to termination of the main GNC loop.
         Since barcSq_ can be different for each factor, we look for the minimimum (positive) quantity in remark 5 in GNC paper
         */
        lambdaInit = std::numeric_limits<double>::infinity();
        for (size_t k = 0; k < nfg_.size(); k++) {
          if (hasNoise(factorTypes_[k])) {
            double rk = nfg_[k]->error(state_);
            lambdaInit = (2 * rk - barcSq_[k]) > 0 ? // if positive, update lambda, otherwise keep same
                std::min(lambdaInit, barcSq_[k] / (2 * rk - barcSq_[k]) ) : lambdaInit;
          }
        }
        if (lambdaInit >= 0 && lambdaInit < 1e-6){
          lambdaInit = 1e-6; // if lambda ~ 0 (but positive), that means we have measurements with large errors,
          // i.e., rk > barcSq_[k] and rk very large, hence we threshold to 1e-6 to avoid lambda = 0
        }
  
        return lambdaInit > 0 && !std::isinf(lambdaInit) ? lambdaInit : -1; // if lambda <= 0 or lambda = inf, return -1,
        // which leads to termination of the main gnc loop. In this case, all residuals are already below the threshold
        // and there is no need to robustify (TLS = least squares)
      default:
        throw std::runtime_error(
            "GncOptimizer::initializeLambda: called with unknown loss type.");
    }
  }

  /// Update the gnc parameter lambda to gradually increase nonconvexity.
  double updateLambda(const double lambda) const {
    switch (params_.lossType) {
      case GncLossType::GM:
        // reduce lambda, but saturate at 1 (original cost is recovered for lambda -> 1)
        return std::max(1.0, lambda / params_.lambdaStep);
      case GncLossType::TLS:
        // increases lambda at each iteration (original cost is recovered for lambda -> inf)
        switch (params_.scheduler) {
          case GncScheduler::SuperLinear: {
            if (lambda < 1) return std::min(std::sqrt(lambda) * params_.lambdaStep, params_.lambdaMax);
            return std::min(lambda * params_.lambdaStep, params_.lambdaMax);
          }
          case GncScheduler::Linear: {
            return lambda * params_.lambdaStep;
          }
          default:
            throw std::runtime_error("GncOptimizer::updateLambda: unknown scheduler type.");
        }
      default:
        throw std::runtime_error(
            "GncOptimizer::updateLambda: called with unknown loss type.");
    }
  }

  /// Check if we have reached the value of lambda for which the surrogate loss matches the original loss.
  bool checkLambdaConvergence(const double lambda) const {
    bool lambdaConverged = false;
    switch (params_.lossType) {
      case GncLossType::GM:
        lambdaConverged = std::fabs(lambda - 1.0) < 1e-9;  // lambda=1 recovers the original GM function
        break;
      case GncLossType::TLS:
        lambdaConverged = false;  // for TLS there is no stopping condition on lambda (it must tend to infinity)
        break;
      default:
        throw std::runtime_error(
            "GncOptimizer::checkLambdaConvergence: called with unknown loss type.");
    }
    if (lambdaConverged && params_.verbosity >= GncParameters::Verbosity::SUMMARY)
      std::cout << "lambdaConverged = true " << std::endl;
    return lambdaConverged;
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
  bool checkConvergence(const double lambda, const Vector& weights,
                        const double cost, const double prev_cost) const {
    return checkCostConvergence(cost, prev_cost)
        || checkWeightsConvergence(weights) || checkLambdaConvergence(lambda);
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

  /** Map this optimizer's historical graduation parameter to the normalized
   * \mu in [0,1] expected by the robust loss interface.
   *
   * \lambda is this class' name for the unbounded control parameter of the
   * schedule, for every loss, which is why the parameters that drive it are
   * named lambdaStep and lambdaMax. It is internal scheduling state, and the
   * quantity it corresponds to in the publications differs per loss:
   *  - GM: \lambda decreases from a large value to 1; \mu = 1 / \lambda.
   *  - TLS: \lambda increases from ~0 to infinity; \mu = \lambda/(1 + \lambda).
   *    This is the same quantity LossFunctions.h calls \theta for TLS.
   * Both maps are exact inverses of the ones documented in LossFunctions.h.
   */
  static double NormalizedMu(GncLossType lossType, const double lambda) {
    switch (lossType) {
      case GncLossType::GM:
        // lambda saturates at 1, which is the final GM loss (mu = 1).
        return std::min(1.0, 1.0 / lambda);
      case GncLossType::TLS:
        return lambda / (1.0 + lambda);
      default:
        throw std::runtime_error(
            "GncOptimizer::NormalizedMu: called with unknown loss type.");
    }
  }

  /// Calculate gnc weights.
  Vector calculateWeights(const Values& currentEstimate, const double lambda) {
    Vector weights = initializeWeightsFromKnownInliersAndOutliers();
    const double mu = NormalizedMu(params_.lossType, lambda);

    // update weights of unknown measurements
    switch (params_.lossType) {
      case GncLossType::GM: {  // use eq (12) in GNC paper
        for (size_t k = 0; k < nfg_.size(); k++) {
          if (needsWeightUpdate(factorTypes_[k])) {
            // squared (and whitened) residual
            double u2_k = nfg_[k]->error(currentEstimate);
            weights[k] = noiseModel::mEstimator::GemanMcClure::GraduatedWeight(
                u2_k, barcSq_[k], mu,
                noiseModel::mEstimator::GemanMcClure::GradScheme::STANDARD);
          }
        }
        return weights;
      }
      case GncLossType::TLS: {
        for (size_t k = 0; k < nfg_.size(); k++) {
          if (needsWeightUpdate(factorTypes_[k])) {
            double u2_k = nfg_[k]->error(
                currentEstimate);  // squared (and whitened) residual
            switch (params_.scheduler) {
              case GncScheduler::SuperLinear: {
                weights[k] = noiseModel::mEstimator::TruncatedLeastSquares::
                    GraduatedWeight(
                        u2_k, barcSq_[k], mu,
                        noiseModel::mEstimator::TruncatedLeastSquares::
                            GradScheme::GNC_SUPERLINEAR);
                break;
              }
              case GncScheduler::Linear: {  // use eq (14) in GNC paper
                weights[k] = noiseModel::mEstimator::TruncatedLeastSquares::
                    GraduatedWeight(
                        u2_k, barcSq_[k], mu,
                        noiseModel::mEstimator::TruncatedLeastSquares::
                            GradScheme::GNC_LINEAR);
                break;
              }
              default:
                throw std::runtime_error(
                    "GncOptimizer::calculateWeights: unknown scheduler type.");
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
