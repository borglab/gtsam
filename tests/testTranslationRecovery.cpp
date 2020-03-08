/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TranslationRecovery.h
 * @author Frank Dellaert
 * @date March 2020
 * @brief test recovering translations when rotations are given.
 */

#include <gtsam/sfm/TranslationFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

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
// or not. Because the latter one is called from the first one, this is prime.

class TranslationRecovery {
 public:
  using KeyPair = std::pair<Key, Key>;
  using TranslationEdges = std::map<KeyPair, Unit3>;

 private:
  TranslationEdges relativeTranslations_;

 public:
  /**
   * @brief Construct a new Translation Recovery object
   *
   * @param relativeTranslations the relative translations, in world coordinate
   * frames, indexed in a map by a pair of Pose keys.
   */
  TranslationRecovery(const TranslationEdges& relativeTranslations)
      : relativeTranslations_(relativeTranslations) {}

  /**
   * @brief Build the factor graph to do the optimization.
   *
   * @return NonlinearFactorGraph
   */
  NonlinearFactorGraph buildGraph() const {
    auto noiseModel = noiseModel::Isotropic::Sigma(3, 0.01);
    NonlinearFactorGraph graph;
    for (auto edge : relativeTranslations_) {
      Key a, b;
      std::tie(a, b) = edge.first;
      const Unit3 w_aZb = edge.second;
      graph.emplace_shared<TranslationFactor>(a, b, w_aZb, noiseModel);
    }
    return graph;
  }

  /**
   * @brief Create random initial translations.
   *
   * @return Values
   */
  Values initalizeRandomly() const {
    Values initial;
    auto insert = [&initial](Key j) {
      if (!initial.exists(j)) {
        initial.insert<Point3>(j, Vector3::Random());
      }
    };

    // Loop over measurements and add a random translation
    for (auto edge : relativeTranslations_) {
      Key a, b;
      std::tie(a, b) = edge.first;
      insert(a);
      insert(b);
    }
    return initial;
  }

  /**
   * @brief Build and optimize factor graph.
   *
   * @param scale scale for first relative translation which fixes gauge.
   * @return Values
   */
  Values run(const double scale = 1.0) const {
    const auto graph = buildGraph();
    const Values initial = initalizeRandomly();

    LevenbergMarquardtParams params;
    params.setVerbosityLM("Summary");
    LevenbergMarquardtOptimizer lm(graph, initial, params);
    Values result = lm.optimize();

    return result;
  }

  /**
   * @brief Simulate translation direction measurements
   *
   * @param poses SE(3) ground truth poses stored as Values
   * @param edges pairs (a,b) for which a measurement w_aZb will be generated.
   */
  static TranslationEdges SimulateMeasurements(
      const Values& poses, const std::vector<KeyPair>& edges) {
    TranslationEdges relativeTranslations;
    for (auto edge : edges) {
      Key a, b;
      std::tie(a, b) = edge;
      const Pose3 wTa = poses.at<Pose3>(a), wTb = poses.at<Pose3>(b);
      const Point3 Ta = wTa.translation(), Tb = wTb.translation();
      const Unit3 w_aZb(Tb - Ta);
      relativeTranslations[edge] = w_aZb;
    }
    return relativeTranslations;
  }
};
}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testTranslationRecovery.cpp
 * @author Frank Dellaert
 * @date March 2020
 * @brief test recovering translations when rotations are given.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/slam/dataset.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// We read the BAL file, which has 3 cameras in it, with poses. We then assume
// the rotations are correct, but translations have to be estimated from
// translation directions only. Since we have 3 cameras, A, B, and C, we can at
// most create three relative measurements, let's call them w_aZb, w_aZc, and
// bZc. These will be of type Unit3. We then call `recoverTranslations` which
// sets up an optimization problem for the three unknown translations.
TEST(TranslationRecovery, BAL) {
  const string filename = findExampleDataFile("dubrovnik-3-7-pre");
  SfM_data db;
  bool success = readBAL(filename, db);
  if (!success) throw runtime_error("Could not access file!");

  // Get camera poses, as Values
  size_t j = 0;
  Values poses;
  for (auto camera : db.cameras) {
    poses.insert(j++, camera.pose());
  }

  // Simulate measurements
  const auto relativeTranslations = TranslationRecovery::SimulateMeasurements(
      poses, {{0, 1}, {0, 2}, {1, 2}});

  // Check
  const Pose3 wTa = poses.at<Pose3>(0), wTb = poses.at<Pose3>(1),
              wTc = poses.at<Pose3>(2);
  const Point3 Ta = wTa.translation(), Tb = wTb.translation(),
               Tc = wTc.translation();
  const Rot3 aRw = wTa.rotation().inverse();
  const Unit3 w_aZb = relativeTranslations.at({0, 1});
  EXPECT(assert_equal(Unit3(Tb - Ta), w_aZb));
  const Unit3 w_aZc = relativeTranslations.at({0, 2});
  EXPECT(assert_equal(Unit3(Tc - Ta), w_aZc));

  TranslationRecovery algorithm(relativeTranslations);
  const auto graph = algorithm.buildGraph();
  EXPECT_LONGS_EQUAL(3, graph.size());

  // Translation recovery, version 1
  const auto result = algorithm.run(2);

  // Check result
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(0)));
  EXPECT(assert_equal(Point3(2 * w_aZb.point3()), result.at<Point3>(1)));
  EXPECT(assert_equal(Point3(0, 0, 0), result.at<Point3>(2)));

  // TODO(frank): how to get stats back
  // EXPECT_DOUBLES_EQUAL(0.0199833, actualError, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
