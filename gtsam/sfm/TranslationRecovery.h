/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TranslationRecovery.h
 * @author Frank Dellaert, Akshay Krishnan
 * @date March 2020
 * @brief Recovering translations in an epipolar graph when rotations are given.
 */

#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/BinaryMeasurement.h>

#include <map>
#include <set>
#include <utility>
#include <vector>

namespace gtsam {

// Set up an optimization problem for the unknown translations Ti in the world
// coordinate frame, given the known camera attitudes wRi with respect to the
// world frame, and a set of (noisy) translation directions of type Unit3,
// w_aZb. The measurement equation is
//    w_aZb = Unit3(Tb - Ta)   (1)
// i.e., w_aZb is the translation direction from frame A to B, in world
// coordinates. Although Unit3 instances live on a manifold, following
// Wilson14eccv_1DSfM.pdf error we compute the *chordal distance* in the
// ambient world coordinate frame.
//
// It is clear that we cannot recover the scale, nor the absolute position,
// so the gauge freedom in this case is 3 + 1 = 4. We fix these by taking fixing
// the translations Ta and Tb associated with the first measurement w_aZb,
// clamping them to their initial values as given to this method. If no initial
// values are given, we use the origin for Tb and set Tb to make (1) come
// through, i.e.,
//    Tb = s * wRa * Point3(w_aZb)     (2)
// where s is an arbitrary scale that can be supplied, default 1.0. Hence, two
// versions are supplied below corresponding to whether we have initial values
// or not.
class GTSAM_EXPORT TranslationRecovery {
 public:
  using KeyPair = std::pair<Key, Key>;
  using TranslationEdges = std::vector<BinaryMeasurement<Unit3>>;

 private:
  // Translation directions between camera pairs.
  TranslationEdges relativeTranslations_;

  // Parameters.
  LevenbergMarquardtParams lmParams_;

  const bool use_bilinear_translation_factor_ = false;

 public:
  /**
   * @brief Construct a new Translation Recovery object
   *
   * @param lmParams parameters for optimization.
   */
  TranslationRecovery(const LevenbergMarquardtParams &lmParams,
                      bool use_bilinear_translation_factor)
      : lmParams_(lmParams),
        use_bilinear_translation_factor_(use_bilinear_translation_factor) {}

  /**
   * @brief Default constructor.
   */
  TranslationRecovery() = default;

  /**
   * @brief Build the factor graph to do the optimization.
   *
   * @param relativeTranslations unit translation directions between
   * translations to be estimated
   * @return NonlinearFactorGraph
   */
  NonlinearFactorGraph buildGraph(
      const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations) const;

  /**
   * @brief Add 3 factors to the graph:
   *    - A prior on the first point to lie at (0, 0, 0)
   *    - If betweenTranslations is non-empty, between factors provided by it.
   *    - If betweenTranslations is empty, a prior on scale of the first
   * relativeTranslations edge.
   *
   * @param relativeTranslations unit translation directions between
   * translations to be estimated
   * @param scale scale for first relative translation which fixes gauge.
   * @param graph factor graph to which prior is added.
   * @param betweenTranslations relative translations (with scale) between 2
   * points in world coordinate frame known a priori.
   * @param priorNoiseModel the noise model to use with the prior.
   */
  void addPrior(
      const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations,
      const double scale,
      const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
      NonlinearFactorGraph *graph,
      const SharedNoiseModel &priorNoiseModel =
          noiseModel::Isotropic::Sigma(3, 0.01)) const;

  /**
   * @brief Create random initial translations.
   *
   * @param relativeTranslations unit translation directions between
   * translations to be estimated
   * @param betweenTranslations relative translations (with scale) between 2
   * points in world coordinate frame known a priori.
   * @param rng random number generator
   * @param intialValues (optional) initial values from a prior
   * @return Values
   */
  Values initializeRandomly(
      const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations,
      const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
      std::mt19937 *rng, const Values &initialValues = Values()) const;

  /**
   * @brief Version of initializeRandomly with a fixed seed.
   *
   * @param relativeTranslations unit translation directions between
   * translations to be estimated
   * @param betweenTranslations relative translations (with scale) between 2
   * points in world coordinate frame known a priori.
   * @param initialValues (optional) initial values from a prior
   * @return Values
   */
  Values initializeRandomly(
      const std::vector<BinaryMeasurement<Unit3>> &relativeTranslations,
      const std::vector<BinaryMeasurement<Point3>> &betweenTranslations,
      const Values &initialValues = Values()) const;

  /**
   * @brief Build and optimize factor graph.
   *
   * @param relativeTranslations the relative translations, in world coordinate
   * frames, vector of BinaryMeasurements of Unit3, where each key of a
   * measurement is a point in 3D. If a relative translation magnitude is zero,
   * it is treated as a hard same-point constraint (the result of all nodes
   * connected by a zero-magnitude edge will be the same).
   * @param scale scale for first relative translation which fixes gauge.
   * The scale is only used if betweenTranslations is empty.
   * @param betweenTranslations relative translations (with scale) between 2
   * points in world coordinate frame known a priori. Unlike
   * relativeTranslations, zero-magnitude betweenTranslations are not treated as
   * hard constraints.
   * @param initialValues intial values for optimization. Initializes randomly
   * if not provided.
   * @return Values
   */
  Values run(
      const TranslationEdges &relativeTranslations, const double scale = 1.0,
      const std::vector<BinaryMeasurement<Point3>> &betweenTranslations = {},
      const Values &initialValues = Values()) const;

  /**
   * @brief Simulate translation direction measurements
   *
   * @param poses SE(3) ground truth poses stored as Values
   * @param edges pairs (a,b) for which a measurement w_aZb will be generated.
   * @return TranslationEdges vector of binary measurements where the keys are
   * the cameras and the measurement is the simulated Unit3 translation
   * direction between the cameras.
   */
  static TranslationEdges SimulateMeasurements(
      const Values &poses, const std::vector<KeyPair> &edges);
};
}  // namespace gtsam
