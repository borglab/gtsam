/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ShonanAveraging.h
 * @date   March 2019 - August 2020
 * @author Frank Dellaert, David Rosen, and Jing Wu
 * @brief  Shonan Averaging algorithm
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/sfm/BinaryMeasurement.h>
#include <gtsam/linear/PowerMethod.h>
#include <gtsam/linear/AcceleratedPowerMethod.h>
#include <gtsam/slam/dataset.h>

#include <Eigen/Sparse>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace gtsam {
class NonlinearFactorGraph;
class LevenbergMarquardtOptimizer;

/// Parameters governing optimization etc.
template <size_t d>
struct GTSAM_EXPORT ShonanAveragingParameters {
  // Select Rot2 or Rot3 interface based template parameter d
  using Rot = typename std::conditional<d == 2, Rot2, Rot3>::type;
  using Anchor = std::pair<size_t, Rot>;

  // Parameters themselves:
  LevenbergMarquardtParams lm;  ///< LM parameters
  double optimalityThreshold;   ///< threshold used in checkOptimality
  Anchor anchor;                ///< pose to use as anchor if not Karcher
  double alpha;                 ///< weight of anchor-based prior (default 0)
  double beta;                  ///< weight of Karcher-based prior (default 1)
  double gamma;                 ///< weight of gauge-fixing factors (default 0)
  /// if enabled, the Huber loss is used (default false)
  bool useHuber;
  /// if enabled solution optimality is certified (default true)
  bool certifyOptimality;

  ShonanAveragingParameters(const LevenbergMarquardtParams &lm =
                                LevenbergMarquardtParams::CeresDefaults(),
                            const std::string &method = "JACOBI",
                            double optimalityThreshold = -1e-4,
                            double alpha = 0.0, double beta = 1.0,
                            double gamma = 0.0);

  LevenbergMarquardtParams getLMParams() const { return lm; }

  void setOptimalityThreshold(double value) { optimalityThreshold = value; }
  double getOptimalityThreshold() const { return optimalityThreshold; }

  void setAnchor(size_t index, const Rot &value) { anchor = {index, value}; }
  std::pair<size_t, Rot> getAnchor() const { return anchor; }

  void setAnchorWeight(double value) { alpha = value; }
  double getAnchorWeight() const { return alpha; }

  void setKarcherWeight(double value) { beta = value; }
  double getKarcherWeight() const { return beta; }

  void setGaugesWeight(double value) { gamma = value; }
  double getGaugesWeight() const { return gamma; }

  void setUseHuber(bool value) { useHuber = value; }
  bool getUseHuber() const { return useHuber; }

  void setCertifyOptimality(bool value) { certifyOptimality = value; }
  bool getCertifyOptimality() const { return certifyOptimality; }

  /// Print the parameters and flags used for rotation averaging.
  void print(const std::string &s = "") const {
    std::cout << (s.empty() ? s : s + " ");
    std::cout << " ShonanAveragingParameters: " << std::endl;
    std::cout << " alpha: " << alpha << std::endl;
    std::cout << " beta: " << beta << std::endl;
    std::cout << " gamma: " << gamma << std::endl;
    std::cout << " useHuber: " << useHuber << std::endl;
  }
};

using ShonanAveragingParameters2 = ShonanAveragingParameters<2>;
using ShonanAveragingParameters3 = ShonanAveragingParameters<3>;

/**
 * Class that implements Shonan Averaging from our ECCV'20 paper.
 * Note: The "basic" API uses all Rot values (Rot2 or Rot3, depending on value
 * of d), whereas the different levels and "advanced" API at SO(p) needs Values
 * of type SOn<Dynamic>.
 *
 * The template parameter d can be 2 or 3.
 * Both are specialized in the .cpp file.
 *
 * If you use this code in your work, please consider citing our paper:
 *    Shonan Rotation Averaging, Global Optimality by Surfing SO(p)^n
 *    Frank Dellaert, David M. Rosen, Jing Wu, Robert Mahony, and Luca Carlone,
 *    European Computer Vision Conference, 2020.
 * You can view our ECCV spotlight video at https://youtu.be/5ppaqMyHtE0
 */
template <size_t d>
class GTSAM_EXPORT ShonanAveraging {
 public:
  using Sparse = Eigen::SparseMatrix<double>;

  // Define the Parameters type and use its typedef of the rotation type:
  using Parameters = ShonanAveragingParameters<d>;
  using Rot = typename Parameters::Rot;

  // We store SO(d) BetweenFactors to get noise model
  using Measurements = std::vector<BinaryMeasurement<Rot>>;

 private:
  Parameters parameters_;
  Measurements measurements_;
  size_t nrUnknowns_;
  Sparse D_;  // Sparse (diagonal) degree matrix
  Sparse Q_;  // Sparse measurement matrix, == \tilde{R} in Eriksson18cvpr
  Sparse L_;  // connection Laplacian L = D - Q, needed for optimality check

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

  /// Construct from set of relative measurements (given as BetweenFactor<Rot3>
  /// for now) NoiseModel *must* be isotropic.
  ShonanAveraging(const Measurements &measurements,
                  const Parameters &parameters = Parameters());

  /// @}
  /// @name Query properties
  /// @{

  /// Return number of unknowns
  size_t nrUnknowns() const { return nrUnknowns_; }

  /// Return number of measurements
  size_t numberMeasurements() const { return measurements_.size(); }

  /// k^th binary measurement
  const BinaryMeasurement<Rot> &measurement(size_t k) const {
    return measurements_[k];
  }

  /**
   * Update factors to use robust Huber loss.
   *
   * @param measurements Vector of BinaryMeasurements.
   * @param k Huber noise model threshold.
   */
  Measurements makeNoiseModelRobust(const Measurements &measurements,
                                    double k = 1.345) const {
    Measurements robustMeasurements;
    for (auto &measurement : measurements) {
      auto model = measurement.noiseModel();
      const auto &robust =
          boost::dynamic_pointer_cast<noiseModel::Robust>(model);

      SharedNoiseModel robust_model;
      // Check if the noise model is already robust
      if (robust) {
        robust_model = model;
      } else {
        // make robust
        robust_model = noiseModel::Robust::Create(
            noiseModel::mEstimator::Huber::Create(k), model);
      }
      BinaryMeasurement<Rot> meas(measurement.key1(), measurement.key2(),
                                  measurement.measured(), robust_model);
      robustMeasurements.push_back(meas);
    }
    return robustMeasurements;
  }

  /// k^th measurement, as a Rot.
  const Rot &measured(size_t k) const { return measurements_[k].measured(); }

  /// Keys for k^th measurement, as a vector of Key values.
  const KeyVector &keys(size_t k) const { return measurements_[k].keys(); }

  /// @}
  /// @name Matrix API (advanced use, debugging)
  /// @{

  Sparse D() const { return D_; }               ///< Sparse version of D
  Matrix denseD() const { return Matrix(D_); }  ///< Dense version of D
  Sparse Q() const { return Q_; }               ///< Sparse version of Q
  Matrix denseQ() const { return Matrix(Q_); }  ///< Dense version of Q
  Sparse L() const { return L_; }               ///< Sparse version of L
  Matrix denseL() const { return Matrix(L_); }  ///< Dense version of L

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

  /// Project to pxdN Stiefel manifold
  static Matrix StiefelElementMatrix(const Values &values);

  /**
   * Compute minimum eigenvalue for optimality check.
   * @param values: should be of type SOn
   */
  double computeMinEigenValue(const Values &values,
                              Vector *minEigenVector = nullptr) const;

  /**
   * Compute minimum eigenvalue with accelerated power method.
   * @param values: should be of type SOn
   */
  double computeMinEigenValueAP(const Values &values,
                                Vector *minEigenVector = nullptr) const;

  /// Project pxdN Stiefel manifold matrix S to Rot3^N
  Values roundSolutionS(const Matrix &S) const;

  /// Create a VectorValues with eigenvector v_i
  static VectorValues TangentVectorValues(size_t p, const Vector &v);

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
  Values initializeWithDescent(
      size_t p, const Values &values, const Vector &minEigenVector,
      double minEigenValue, double gradienTolerance = 1e-2,
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
   * Create initial Values of type SO(p)
   * @param p the dimensionality of the rotation manifold
   * @param rng random number generator
   */
  Values initializeRandomlyAt(size_t p, std::mt19937 &rng) const;

  /// Version of initializeRandomlyAt with fixed random seed.
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
   * @param initial initial SO(p) values
   * @return lm optimizer
   */
  std::shared_ptr<LevenbergMarquardtOptimizer> createOptimizerAt(
      size_t p, const Values &initial) const;

  /**
   * Try to optimize at SO(p)
   * @param p the dimensionality of the rotation manifold to optimize over
   * @param initial initial SO(p) values
   * @return SO(p) values
   */
  Values tryOptimizingAt(size_t p, const Values &initial) const;

  /**
   * Project from SO(p) to Rot2 or Rot3
   * Values should be of type SO(p)
   */
  Values projectFrom(size_t p, const Values &values) const;

  /**
   * Project from SO(p)^N to Rot2^N or Rot3^N
   * Values should be of type SO(p)
   */
  Values roundSolution(const Values &values) const;

  /// Lift Values of type T to SO(p)
  template <class T>
  static Values LiftTo(size_t p, const Values &values) {
    Values result;
    for (const auto it : values.extract<T>()) {
      result.insert(it.first, SOn::Lift(p, it.second.matrix()));
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
   * Initialize randomly at SO(d)
   * @param rng random number generator
   * Example:
   *   std::mt19937 rng(42);
   *   Values initial = initializeRandomly(rng, p);
   */
  Values initializeRandomly(std::mt19937 &rng) const;

  /// Random initialization for wrapper, fixed random seed.
  Values initializeRandomly() const;

  /**
   * Optimize at different values of p until convergence.
   * @param initial initial Rot3 values
   * @param pMin value of p to start Riemanian staircase at (default: d).
   * @param pMax maximum value of p to try (default: 10)
   * @return (Rot3 values, minimum eigenvalue)
   */
  std::pair<Values, double> run(const Values &initialEstimate, size_t pMin = d,
                                size_t pMax = 10) const;
  /// @}

  /**
   * Helper function to convert measurements to robust noise model
   * if flag is set.
   *
   * @tparam T the type of measurement, e.g. Rot3.
   * @param measurements vector of BinaryMeasurements of type T.
   * @param useRobustModel flag indicating whether use robust noise model
   * instead.
   */
  template <typename T>
  inline std::vector<BinaryMeasurement<T>> maybeRobust(
      const std::vector<BinaryMeasurement<T>> &measurements,
      bool useRobustModel = false) const {
    return useRobustModel ? makeNoiseModelRobust(measurements) : measurements;
  }
};

// Subclasses for d=2 and d=3 that explicitly instantiate, as well as provide a
// convenience interface with file access.

class GTSAM_EXPORT ShonanAveraging2 : public ShonanAveraging<2> {
 public:
  ShonanAveraging2(const Measurements &measurements,
                   const Parameters &parameters = Parameters());
  explicit ShonanAveraging2(std::string g2oFile,
                            const Parameters &parameters = Parameters());
  ShonanAveraging2(const BetweenFactorPose2s &factors,
                   const Parameters &parameters = Parameters());
};

class GTSAM_EXPORT ShonanAveraging3 : public ShonanAveraging<3> {
 public:
  ShonanAveraging3(const Measurements &measurements,
                   const Parameters &parameters = Parameters());
  explicit ShonanAveraging3(std::string g2oFile,
                            const Parameters &parameters = Parameters());

  // TODO(frank): Deprecate after we land pybind wrapper
  ShonanAveraging3(const BetweenFactorPose3s &factors,
                   const Parameters &parameters = Parameters());
};
}  // namespace gtsam
