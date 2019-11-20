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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/slam/dataset.h>

#include <Eigen/Sparse>

#include <map>
#include <string>

namespace gtsam {
class NonlinearFactorGraph;

/// Parameters governing optimization etc.
struct ShonanAveragingParameters {
  bool prior;                   // whether to use a prior
  bool karcher;                 // whether to use Karcher mean prior
  double noiseSigma;            // Optional noise Sigma, will be ignored if zero
  double optimalityThreshold;   // threshold used in checkOptimality
  LevenbergMarquardtParams lm;  // LM parameters
  ShonanAveragingParameters(const std::string& verbosity = "SILENT",
                            const std::string& method = "JACOBI",
                            double noiseSigma = 0,
                            double optimalityThreshold = -1e-4);
  void setPrior(bool value) { prior = value; }
  void setKarcher(bool value) { karcher = value; }
  void setNoiseSigma(bool value) { noiseSigma = value; }
};

class ShonanAveraging {
 public:
  using Sparse = Eigen::SparseMatrix<double>;

 private:
  ShonanAveragingParameters parameters_;
  BetweenFactorPose3s factors_;
  std::map<Key, Pose3> poses_;
  Sparse L_;  // connection Laplacian needed for optimality check
  size_t d_;  // dimensionality (typically 2 or 3)

 public:
  /**
   * Construct from a G2O file
   */
  explicit ShonanAveraging(const std::string& g2oFile,
                           const ShonanAveragingParameters& parameters =
                               ShonanAveragingParameters());

  /// Return number of poses
  size_t nrPoses() const { return poses_.size(); }

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
  double costAt(size_t p, const Values& values) const;

  /**
   * Build 3Nx3N sparse matrix consisting of rotation measurements, arranged as
   *       (i,j) and (j,i) blocks within a sparse matrix.
   * @param useNoiseModel whether to use noise model
   */
  Sparse buildQ(bool useNoiseModel = false) const;

  /**
   * Given an estimated local minimum Yopt for the (possibly lifted)
   * relaxation, this function computes and returns the block-diagonal elements
   * of the corresponding Lagrange multiplier.
   */
  Sparse computeLambda(const Values& values) const;

  // Version that takes pxdN Stiefel manifold elements
  Sparse computeLambda(const Matrix& S) const;

  /** 
   * Compute minimum eigenvalue for optimality check.
   * @param values: should be of type SOn
   */
  double computeMinEigenValue(const Values& values, Vector* minEigenVector = nullptr) const;

  /**
   * Check optimality
   * @param values: should be of type SOn
   */
  bool checkOptimality(const Values& values) const;

  /**
   * Try to optimize at SO(p)
   * @param p the dimensionality of the rotation manifold to optimize over
   * @param initial optional initial SO(p) values
   * @return SO(p) values
   */
  Values tryOptimizingAt(
      size_t p,
      const boost::optional<const Values&> initial = boost::none) const;

  /**
   * Project from SO(p) to SO(3)
   * Values should be of type SO(p)
   */
  Values projectFrom(size_t p, const Values& values) const;

  /**
   * Project from SO(p)^N to SO(3)^N
   * Values should be of type SO(p)
   */
  Values roundSolution(const Matrix Y) const;
  Values roundSolution(const Values& values) const;

  /**
   * Calculate cost for SO(3)
   * Values should be of type SO3
   */
  double cost(const Values& values) const;

  /** 
   * Given some values at p, return new values at p+1, by doing a line search along the descent direction, computed from the minimum eigenvector at p.
   * @param values should be of type SO(p)
   * @param minEigenVector corresponding to minEigenValue at level p
   * @return values of type SO(p+1)
   */
  Values initializeWithDescent(const Values& values, const Vector& minEigenVector) const;

  /**
   * Optimize at different values of p until convergence, with random init at each level.
   * @param pMin value of p to start Riemanian staircase at.
   * @param pMax maximum value of p to try (default: 20)
   * @param withDescent use descent direction from paper.
   * @return (SO(3) values, minimum eigenvalue)
   */
  std::pair<Values, double> run(size_t pMin, size_t pMax, bool withDescent) const;

  /**
   * Optimize at different values of p until convergence, with random init at each level.
   */
  std::pair<Values, double> runWithRandom(size_t pMin = 5, size_t pMax = 20) const;

  /**
   * Optimize at different values of p until convergence, with descent direction.
   */
  std::pair<Values, double> runWithDescent(size_t pMin = 5, size_t pMax = 20) const;
};

}  // namespace gtsam
