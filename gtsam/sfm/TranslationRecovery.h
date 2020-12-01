/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TranslationRecovery.h
 * @author Frank Dellaert
 * @date March 2020
 * @brief Recovering translations in an epipolar graph when rotations are given.
 */

#include <map>
#include <set>
#include <utility>
#include <vector>

#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/BinaryMeasurement.h>

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
class TranslationRecovery {
 public:
  using KeyPair = std::pair<Key, Key>;
  using TranslationEdges = std::vector<BinaryMeasurement<Unit3>>;

 private:
  // Translation directions between camera pairs.
  TranslationEdges relativeTranslations_;

  // Parameters used by the LM Optimizer.
  LevenbergMarquardtParams params_;

  // Map from a key in the graph to a set of keys that share the same
  // translation.
  std::map<Key, std::set<Key>> sameTranslationNodes_;

 public:
  /**
   * @brief Construct a new Translation Recovery object
   *
   * @param relativeTranslations the relative translations, in world coordinate
   * frames, vector of BinaryMeasurements of Unit3, where each key of a
   * measurement is a point in 3D.
   * @param lmParams (optional) gtsam::LavenbergMarquardtParams that can be
   * used to modify the parameters for the LM optimizer. By default, uses the
   * default LM parameters.
   */
  TranslationRecovery(
      const TranslationEdges &relativeTranslations,
      const LevenbergMarquardtParams &lmParams = LevenbergMarquardtParams());

  /**
   * @brief Build the factor graph to do the optimization.
   *
   * @return NonlinearFactorGraph
   */
  NonlinearFactorGraph buildGraph() const;

  /**
   * @brief Add priors on ednpoints of first measurement edge.
   *
   * @param scale scale for first relative translation which fixes gauge.
   * @param graph factor graph to which prior is added.
   * @param priorNoiseModel the noise model to use with the prior.
   */
  void addPrior(const double scale, NonlinearFactorGraph *graph,
                const SharedNoiseModel &priorNoiseModel =
                    noiseModel::Isotropic::Sigma(3, 0.01)) const;

  /**
   * @brief Create random initial translations.
   *
   * @return Values
   */
  Values initalizeRandomly() const;

  /**
   * @brief Build and optimize factor graph.
   *
   * @param scale scale for first relative translation which fixes gauge.
   * @return Values
   */
  Values run(const double scale = 1.0) const;

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
