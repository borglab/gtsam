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

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
/// Choice of robust loss function for GNC.
enum GncLossType {
  GM /*Geman McClure*/,
  TLS /*Truncated least squares*/
};

template<class BaseOptimizerParameters>
class GncParams {
 public:
  /// For each parameter, specify the corresponding optimizer: e.g., GaussNewtonParams -> GaussNewtonOptimizer.
  typedef typename BaseOptimizerParameters::OptimizerType OptimizerType;

  /// Verbosity levels
  enum Verbosity {
    SILENT = 0,
    SUMMARY,
    MU,
    WEIGHTS,
    VALUES
  };

  /// Constructor.
  GncParams(const BaseOptimizerParameters& baseOptimizerParams)
      : baseOptimizerParams(baseOptimizerParams) {
  }

  /// Default constructor.
  GncParams()
      : baseOptimizerParams() {
  }

  /// GNC parameters.
  BaseOptimizerParameters baseOptimizerParams;  ///< Optimization parameters used to solve the weighted least squares problem at each GNC iteration
  /// any other specific GNC parameters:
  GncLossType lossType = TLS;  ///< Default loss
  size_t maxIterations = 100;  ///<  Maximum number of iterations
  double muStep = 1.4;  ///< Multiplicative factor to reduce/increase the mu in gnc
  double relativeCostTol = 1e-5;  ///< If relative cost change is below this threshold, stop iterating
  double weightsTol = 1e-4;  ///< If the weights are within weightsTol from being binary, stop iterating (only for TLS)
  Verbosity verbosity = SILENT;  ///< Verbosity level

  /// Use IndexVector for inliers and outliers since it is fast
  using IndexVector = FastVector<uint64_t>;
  ///< Slots in the factor graph corresponding to measurements that we know are inliers
  IndexVector knownInliers;
  ///< Slots in the factor graph corresponding to measurements that we know are outliers
  IndexVector knownOutliers;

  /// Set the robust loss function to be used in GNC (chosen among the ones in GncLossType).
  void setLossType(const GncLossType type) {
    lossType = type;
  }

  /// Set the maximum number of iterations in GNC (changing the max nr of iters might lead to less accurate solutions and is not recommended).
  void setMaxIterations(const size_t maxIter) {
    std::cout
        << "setMaxIterations: changing the max nr of iters might lead to less accurate solutions and is not recommended! "
        << std::endl;
    maxIterations = maxIter;
  }

  /// Set the graduated non-convexity step: at each GNC iteration, mu is updated as mu <- mu * muStep.
  void setMuStep(const double step) {
    muStep = step;
  }

  /// Set the maximum relative difference in mu values to stop iterating.
  void setRelativeCostTol(double value) {
    relativeCostTol = value;
  }

  /// Set the maximum difference between the weights and their rounding in {0,1} to stop iterating.
  void setWeightsTol(double value) {
    weightsTol = value;
  }

  /// Set the verbosity level.
  void setVerbosityGNC(const Verbosity value) {
    verbosity = value;
  }

  /** (Optional) Provide a vector of measurements that must be considered inliers. The enties in the vector
   * corresponds to the slots in the factor graph. For instance, if you have a nonlinear factor graph nfg,
   * and you provide  knownIn = {0, 2, 15}, GNC will not apply outlier rejection to nfg[0], nfg[2], and nfg[15].
   * This functionality is commonly used in SLAM when one may assume the odometry is outlier free, and
   * only apply GNC to prune outliers from the loop closures.
   * */
  void setKnownInliers(const IndexVector& knownIn) {
    for (size_t i = 0; i < knownIn.size(); i++){
      knownInliers.push_back(knownIn[i]);
    }
    std::sort(knownInliers.begin(), knownInliers.end());
  }

  /** (Optional) Provide a vector of measurements that must be considered outliers. The enties in the vector
   * corresponds to the slots in the factor graph. For instance, if you have a nonlinear factor graph nfg,
   * and you provide  knownOut = {0, 2, 15}, GNC will not apply outlier rejection to nfg[0], nfg[2], and nfg[15].
   * */
  void setKnownOutliers(const IndexVector& knownOut) {
    for (size_t i = 0; i < knownOut.size(); i++){
      knownOutliers.push_back(knownOut[i]);
    }
    std::sort(knownOutliers.begin(), knownOutliers.end());
  }

  /// Equals.
  bool equals(const GncParams& other, double tol = 1e-9) const {
    return baseOptimizerParams.equals(other.baseOptimizerParams)
        && lossType == other.lossType && maxIterations == other.maxIterations
        && std::fabs(muStep - other.muStep) <= tol
        && verbosity == other.verbosity && knownInliers == other.knownInliers
        && knownOutliers == other.knownOutliers;
  }

  /// Print.
  void print(const std::string& str) const {
    std::cout << str << "\n";
    switch (lossType) {
      case GM:
        std::cout << "lossType: Geman McClure" << "\n";
        break;
      case TLS:
        std::cout << "lossType: Truncated Least-squares" << "\n";
        break;
      default:
        throw std::runtime_error("GncParams::print: unknown loss type.");
    }
    std::cout << "maxIterations: " << maxIterations << "\n";
    std::cout << "muStep: " << muStep << "\n";
    std::cout << "relativeCostTol: " << relativeCostTol << "\n";
    std::cout << "weightsTol: " << weightsTol << "\n";
    std::cout << "verbosity: " << verbosity << "\n";
    for (size_t i = 0; i < knownInliers.size(); i++)
      std::cout << "knownInliers: " << knownInliers[i] << "\n";
    for (size_t i = 0; i < knownOutliers.size(); i++)
      std::cout << "knownOutliers: " << knownOutliers[i] << "\n";
    baseOptimizerParams.print("Base optimizer params: ");
  }
};

}
