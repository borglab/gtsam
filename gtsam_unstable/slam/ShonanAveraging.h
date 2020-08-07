/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ShonanAveraging.h
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Shonan Averaging algorithm
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/dllexport.h>

#include <Eigen/Sparse>
#include <map>
#include <string>
#include <utility>

namespace gtsam {
class NonlinearFactorGraph;

/// Parameters governing optimization etc.
struct GTSAM_UNSTABLE_EXPORT ShonanAveragingParameters {
  using Anchor = std::pair<size_t, Rot3>;
  LevenbergMarquardtParams lm; // LM parameters
  double optimalityThreshold;  // threshold used in checkOptimality
  Anchor anchor;               // pose to use as anchor if not Karcher
  double alpha;                // weight of anchor-based prior (default 0)
  double beta;                 // weight of Karcher-based prior (default 1)
  double gamma;                // weight of gauge-fixing factors (default 0)

  ShonanAveragingParameters(const LevenbergMarquardtParams &lm =
                                LevenbergMarquardtParams::CeresDefaults(),
                            const std::string &method = "JACOBI",
                            double optimalityThreshold = -1e-4,
                            double alpha = 0.0, double beta = 1.0,
                            double gamma = 0.0);

  LevenbergMarquardtParams getLMParams() const { return lm; }

  void setOptimalityThreshold(double value) { optimalityThreshold = value; }
  double getOptimalityThreshold() const { return optimalityThreshold; }

  void setAnchor(size_t index, const gtsam::Rot3 &value) {
    anchor = {index, value};
  }

  void setAnchorWeight(double value) { alpha = value; }
  double getAnchorWeight() { return alpha; }

  void setKarcherWeight(double value) { beta = value; }
  double getKarcherWeight() { return beta; }

  void setGaugesWeight(double value) { gamma = value; }
  double getGaugesWeight() { return gamma; }
};

/**
 * Class that implements Shonan Averaging from our ECCV'20 paper. Please cite!
 * Note: The "basic" API uses all Rot3 values, whereas the different levels and
 * "advanced" API at SO(p) needs Values of type SOn<Dynamic>.
 *
 * The template parameter d is typically 2 or 3.
 * Currently d=3 is specialized in the .cpp file.
 */
template <size_t d>
class GTSAM_UNSTABLE_EXPORT ShonanAveraging {
public:
  using Sparse = Eigen::SparseMatrix<double>;

private:
  ShonanAveragingParameters parameters_;
  BetweenFactorPose3s factors_;
  std::map<Key, Pose3> poses_;
  Sparse D_; // Sparse (diagonal) degree matrix
  Sparse Q_; // Sparse measurement matrix, == \tilde{R} in Eriksson18cvpr
  Sparse L_; // connection Laplacian L = D - Q, needed for optimality check

  /**
   * Build 3Nx3N sparse matrix consisting of rotation measurements, arranged as
   * (i,j) and (j,i) blocks within a sparse matrix.
   */
  Sparse buildQ() const;

  /// Build 3Nx3N sparse degree matrix D
  Sparse buildD() const;

public:
  /// @name Standard Constructors
  /// @{

  /// Construct from factors and poses.
  ShonanAveraging(const BetweenFactorPose3s &factors,
                  const std::map<Key, Pose3> &poses,
                  const ShonanAveragingParameters &parameters =
                      ShonanAveragingParameters());

  /// Construct from factors and values (will ignore all but Pose3 values).
  ShonanAveraging(const BetweenFactorPose3s &factors, const Values &values,
                  const ShonanAveragingParameters &parameters =
                      ShonanAveragingParameters());

  /// Construct from a G2O file.
  explicit ShonanAveraging(const std::string &g2oFile,
                           const ShonanAveragingParameters &parameters =
                               ShonanAveragingParameters());

  /// @}
  /// @name Query properties
  /// @{

  /// Return number of poses
  size_t nrPoses() const { return poses_.size(); }

  /// Return number of measurements
  size_t nrMeasurements() const { return factors_.size(); }

  /// k^th measurement, as a Pose3.
  const Pose3 &measured(size_t i) const { return factors_[i]->measured(); }

  /// Keys for k^th measurement, as a vector of Key values.
  const KeyVector &keys(size_t i) const { return factors_[i]->keys(); }

  /// Return poses
  const std::map<Key, Pose3> &poses() const { return poses_; }

  /// @}
  /// @name Matrix API (advanced use, debugging)
  /// @{

  Sparse D() const { return D_; }              ///< Sparse version of D
  Matrix denseD() const { return Matrix(D_); } ///< Dense version of D
  Sparse Q() const { return Q_; }              ///< Sparse version of Q
  Matrix denseQ() const { return Matrix(Q_); } ///< Dense version of Q
  Sparse L() const { return L_; }              ///< Sparse version of L
  Matrix denseL() const { return Matrix(L_); } ///< Dense version of L

  /// Version that takes pxdN Stiefel manifold elements
  Sparse computeLambda(const Matrix &S) const;

  /// Dense versions of computeLambda for wrapper/testing
  Matrix computeLambda_(const Values &values) const {
    return Matrix(computeLambda(values));
  }

  /// Dense versions of computeLambda for wrapper/testing
  Matrix computeLambda_(const Matrix &S) const {
    return Matrix(computeLambda(S));
  }

  /// Compute A matrix whose Eigenvalues we will examine
  Sparse computeA(const Values &values) const;

  /// Version that takes pxdN Stiefel manifold elements
  Sparse computeA(const Matrix &S) const;

  /// Dense version of computeA for wrapper/testing
  Matrix computeA_(const Values &values) const {
    return Matrix(computeA(values));
  }

  /**
   * Compute minimum eigenvalue for optimality check.
   * @param values: should be of type SOn
   */
  double computeMinEigenValue(const Values &values,
                              Vector *minEigenVector = nullptr) const;

  /// Project pxdN Stiefel manifold matrix S to Rot3^N
  Values roundSolutionS(const Matrix &S) const;

  /// Create a tangent direction xi with eigenvector segment v_i
  static Vector MakeATangentVector(size_t p, const Vector &v, size_t i);

  /// Calculate the riemannian gradient of F(values) at values
  Matrix riemannianGradient(size_t p, const Values &values) const;

  /**
   * Lift up the dimension of values in type SO(p-1) with descent direction
   * provided by minEigenVector and return new values in type SO(p)
   */
  static Values LiftwithDescent(size_t p, const Values &values,
                                const Vector &minEigenVector);

  /**
   * Given some values at p-1, return new values at p, by doing a line search
   * along the descent direction, computed from the minimum eigenvector at p-1.
   * @param values should be of type SO(p-1)
   * @param minEigenVector corresponding to minEigenValue at level p-1
   * @return values of type SO(p)
   */
  Values
  initializeWithDescent(size_t p, const Values &values,
                        const Vector &minEigenVector, double minEigenValue,
                        double gradienTolerance = 1e-2,
                        double preconditionedGradNormTolerance = 1e-4) const;
  /// @}
  /// @name Advanced API
  /// @{

  /**
   * Build graph for SO(p)
   * @param p the dimensionality of the rotation manifold to optimize over
   */
  NonlinearFactorGraph buildGraphAt(size_t p) const;

  /**
   * Initialize randomly at SO(p)
   * @param p the dimensionality of the rotation manifold to optimize over
   */
  Values initializeRandomlyAt(size_t p) const;

  /**
   * Calculate cost for SO(p)
   * Values should be of type SO(p)
   */
  double costAt(size_t p, const Values &values) const;

  /**
   * Given an estimated local minimum Yopt for the (possibly lifted)
   * relaxation, this function computes and returns the block-diagonal elements
   * of the corresponding Lagrange multiplier.
   */
  Sparse computeLambda(const Values &values) const;

  /**
   * Compute minimum eigenvalue for optimality check.
   * @param values: should be of type SOn
   * @return minEigenVector and minEigenValue
   */
  std::pair<double, Vector> computeMinEigenVector(const Values &values) const;

  /**
   * Check optimality
   * @param values: should be of type SOn
   */
  bool checkOptimality(const Values &values) const;

  /**
   * Try to create optimizer at SO(p)
   * @param p the dimensionality of the rotation manifold to optimize over
   * @param initial optional initial SO(p) values
   * @return lm optimizer
   */
  boost::shared_ptr<LevenbergMarquardtOptimizer>
  createOptimizerAt(size_t p, const boost::optional<Values> &shonan) const;

  /**
   * Try to optimize at SO(p)
   * @param p the dimensionality of the rotation manifold to optimize over
   * @param initial optional initial SO(p) values
   * @return SO(p) values
   */
  Values
  tryOptimizingAt(size_t p,
                  const boost::optional<Values> &initial = boost::none) const;

  /**
   * Project from SO(p) to Rot3
   * Values should be of type SO(p)
   */
  Values projectFrom(size_t p, const Values &values) const;

  /**
   * Project from SO(p)^N to Rot3^N
   * Values should be of type SO(p)
   */
  Values roundSolution(const Values &values) const;

  /// Lift Values of type T to SO(p)
  template <class T> static Values LiftTo(size_t p, const Values &values) {
    Values result;
    for (const auto &it : values.filter<T>()) {
      result.insert(it.key, SOn::Lift(p, it.value.matrix()));
    }
    return result;
  }

  /// @}
  /// @name Basic API
  /// @{

  /**
   * Calculate cost for SO(3)
   * Values should be of type Rot3
   */
  double cost(const Values &values) const;

  /**
   * Optimize at different values of p until convergence.
   * @param pMin value of p to start Riemanian staircase at.
   * @param pMax maximum value of p to try (default: 20)
   * @param withDescent use descent direction from paper.
   * @param initial optional initial Rot3 values
   * @return (Rot3 values, minimum eigenvalue)
   */
  std::pair<Values, double>
  run(size_t pMin, size_t pMax, bool withDescent,
      const boost::optional<Values> &initialEstimate = boost::none) const;

  /**
   * Optimize at different values of p until convergence, with random
   * initialization at each level.
   *
   * @param pMin value of p to start Riemanian staircase at.
   * @param pMax maximum value of p to try (default: 20)
   * @param initial optional initial Rot3 values
   * @return (Rot3 values, minimum eigenvalue)
   */
  std::pair<Values, double> runWithRandom(
      size_t pMin = 5, size_t pMax = 20,
      const boost::optional<Values> &initialEstimate = boost::none) const;

  /**
   * Optimize at different values of p until convergence, with descent
   * direction derived from Eigenvalue computation.
   *
   * @param pMin value of p to start Riemanian staircase at.
   * @param pMax maximum value of p to try (default: 20)
   * @param initial optional initial Rot3 values
   * @return (Rot3 values, minimum eigenvalue)
   */
  std::pair<Values, double> runWithDescent(
      size_t pMin = 5, size_t pMax = 20,
      const boost::optional<Values> &initialEstimate = boost::none) const;
  /// @}
};

using ShonanAveraging3 = ShonanAveraging<3>;

} // namespace gtsam
