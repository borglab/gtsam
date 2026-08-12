/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

/**
 * @file LocationRecovery.h
 * @author Kathir Gounder, Akshay Krishnan, Frank Dellaert
 * @date April 2026
 * @brief Recover absolute Point3 locations from pairwise Unit3 direction
 *        measurements, using either chordal or bilinear (BATA) cost functions.
 */

#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/BinaryMeasurement.h>

#include <random>
#include <set>
#include <vector>

namespace gtsam {

// Recover absolute Point3 locations from pairwise Unit3 direction
// measurements. This base class is unopinionated about graph structure:
// measurements can connect any pair of Point3 unknowns (cameras, landmarks,
// sensors, etc.). Subclasses add problem-specific opinions:
//   - GlobalPositioner: bipartite camera+landmark, anchor-only gauge.
//   - TranslationRecovery: homogeneous camera-camera, DSFMap, two-key gauge.
//
// Supports two cost functions via the bilinear flag:
//   false — TranslationFactor: normalized(Tb-Ta) - measured  (chordal)
//   true  — BilinearAngleTranslationFactor: scale*(Tb-Ta) - measured  (BATA)
class GTSAM_EXPORT LocationRecovery {
 public:
  using DirectionEdges = std::vector<BinaryMeasurement<Unit3>>;

 protected:
  LevenbergMarquardtParams lmParams_;

  /// Convert Unit3 (2D manifold) noise to Point3 (3D ambient) noise.
  static SharedNoiseModel convertNoiseModel(
      const SharedNoiseModel &unit3NoiseModel);

 public:
  /**
   * @brief Construct with LM parameters.
   */
  explicit LocationRecovery(const LevenbergMarquardtParams &lmParams)
      : lmParams_(lmParams) {}

  /**
   * @brief Default constructor.
   */
  LocationRecovery() = default;

  /**
   * @brief Build factor graph from direction measurements.
   *
   * If bilinear is true, uses BilinearAngleTranslationFactor (BATA) with
   * per-observation scale variables Symbol('S', i). Otherwise uses the
   * chordal TranslationFactor.
   *
   * @param edges    Unit3 direction measurements between Point3 unknowns.
   * @param bilinear if true, use BATA factors (default: true).
   * @return NonlinearFactorGraph
   */
  NonlinearFactorGraph buildGraph(const DirectionEdges &edges,
                                  bool bilinear = true) const;

  /**
   * @brief Add a prior pinning one key to the origin.
   *
   * Fixes 3 translation DOF. Does not assume the key is a camera or
   * landmark — it just pins a Point3.
   */
  void addAnchorPrior(
      Key anchorKey, NonlinearFactorGraph *graph,
      const SharedNoiseModel &priorNoiseModel =
          noiseModel::Isotropic::Sigma(3, 0.01)) const;

  /**
   * @brief Random initialization of Point3 keys and BATA scale variables.
   *
   * Point3 keys are initialized uniformly in [-1, 1]^3. If bilinear is
   * true, also creates numEdges scale variables initialized to 1.0.
   *
   * @param keys          Point3 keys to initialize.
   * @param numEdges      number of edges (for scale variable count).
   * @param bilinear      whether to create BATA scale variables.
   * @param rng           random number generator.
   * @param initialValues optional seed values for specific keys.
   * @return Values
   */
  Values initializeRandomly(const std::set<Key> &keys, size_t numEdges,
                             bool bilinear, std::mt19937 *rng,
                             const Values &initialValues = Values()) const;

  /**
   * @brief Version of initializeRandomly with a fixed seed.
   */
  Values initializeRandomly(const std::set<Key> &keys, size_t numEdges,
                             bool bilinear,
                             const Values &initialValues = Values()) const;
};

}  // namespace gtsam
