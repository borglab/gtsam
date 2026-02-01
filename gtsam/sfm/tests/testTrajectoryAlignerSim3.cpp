/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testTrajectoryAlignerSim3.cpp
 * @author Akshay Krishnan
 * @date January 2026
 * @brief Unit tests for the TrajectoryAlignerSim3 class.
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/TrajectoryAlignerSim3.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/utilities.h>

#include <random>
#include <vector>

using namespace gtsam;
using PoseMeasurements = std::vector<UnaryMeasurement<Pose3>>;
using ChildrenPoses = std::vector<std::vector<UnaryMeasurement<Pose3>>>;

namespace {

Values makeParentValues() {
  Values values;
  values.insert(0, Pose3::Identity());
  values.insert(1, Pose3(Rot3::RzRyRx(0.15, -0.2, 0.1), Point3(1.0, 0.1, 0.0)));
  values.insert(2, Pose3(Rot3::RzRyRx(0.1, 0.05, -0.03), Point3(2.0, 0.4, 0.1)));
  return values;
}

// Makes a vector of unary measurements from a Values of poses.
PoseMeasurements makeMeasurements(const Values& poses, double noiseSigma = 1e-3) {
  PoseMeasurements m;
  auto noise = noiseModel::Isotropic::Sigma(6, noiseSigma);
  for (const auto& key_value : poses) {
    const Key key = key_value.key;
    m.emplace_back(key, poses.at<Pose3>(key), noise);
  }
  return m;
}

// Transforms values of poses by a similarity transform.
Values transformValues(const Similarity3& sim, const Values& poses) {
  Values out;
  for (const auto& key_value : poses) {
    const Key key = key_value.key;
    out.insert(key, sim.transformFrom(poses.at<Pose3>(key)));
  }
  return out;
}

// Perturbs a Values of poses by a small noise.
Values perturbPoses(const Values& poses) {
  Values perturbed = poses;
  utilities::perturbPose3(perturbed, /*sigmaT=*/0.01, /*sigmaR=*/0.01);
  return perturbed;
}

// Perturbs a similarity transform by a large value.
// The initial similarity estimate can be very inaccurate.
Similarity3 perturbSim3(const Similarity3& sim) {
  Similarity3 delta(Rot3::RzRyRx(0.2, -0.25, 0.1), Point3(2, 3, -1),
                    2.3);
  return sim * delta;
}

// Ground truth similarity transforms for the tests.
const Similarity3 gtSim1(Rot3::RzRyRx(0.2, 0.1, -0.05), Point3(0.3, -0.1, 0.2),
                         1.5);
const Similarity3 gtSim2(Rot3::RzRyRx(-0.15, 0.05, 0.08),
                         Point3(-0.2, 0.15, 0.25), 1.3);
const Similarity3 gtSim3(Rot3::RzRyRx(0.05, -0.08, 0.12),
                          Point3(0.15, 0.05, -0.1), 1.1);

// Helper function to check if two similarity transforms are close.
bool simClose(const Similarity3& expected, const Similarity3& actual,
              double tol) {
  return assert_equal<Similarity3>(expected, actual, tol);
}

}  // namespace

/* ************************************************************************* */
TEST(TrajectoryAlignerSim3, PerfectSingleChild) {
  const auto parent = makeParentValues();
  const auto child = transformValues(gtSim1, parent);

  PoseMeasurements aTi = makeMeasurements(parent);
  ChildrenPoses bTi_all{makeMeasurements(child)};
  std::vector<Similarity3> sims{gtSim1};

  TrajectoryAlignerSim3 aligner(aTi, bTi_all, sims);
  Values result = aligner.solve();

  const auto recoveredSim = result.at<Similarity3>(Symbol('S', 0));
  EXPECT(simClose(gtSim1, recoveredSim, 1e-6));

  for (const auto& kv : parent) {
    const Key key = kv.key;
    EXPECT(assert_equal<Pose3>(parent.at<Pose3>(key), result.at<Pose3>(key), 1e-6));
  }
}

/* ************************************************************************* */
TEST(TrajectoryAlignerSim3, PerfectSingleChildNoInitialSim) {
  const auto parent = makeParentValues();
  const Similarity3 gtSim(Rot3::RzRyRx(0.25, -0.05, 0.12),
                          Point3(-0.3, 0.2, -0.15), 1.4);
  const auto child = transformValues(gtSim, parent);

  PoseMeasurements aTi = makeMeasurements(parent);
  ChildrenPoses bTi_all{makeMeasurements(child)};

  TrajectoryAlignerSim3 aligner(aTi, bTi_all);
  Values result = aligner.solve();

  const auto recoveredSim = result.at<Similarity3>(Symbol('S', 0));
  EXPECT(simClose(gtSim, recoveredSim, 1e-6));
}

/* ************************************************************************* */
TEST(TrajectoryAlignerSim3, NoisySingleChild) {
  const auto parent = makeParentValues();
  const auto child = transformValues(gtSim1, parent);
  const auto perturbedChild = perturbPoses(child);

  PoseMeasurements aTi = makeMeasurements(parent, /*noiseSigma=*/1e-2);
  PoseMeasurements bTi = makeMeasurements(perturbedChild, 1e-1);
  ChildrenPoses bTi_all{bTi};
  std::vector<Similarity3> sims{perturbSim3(gtSim1)};

  TrajectoryAlignerSim3 aligner(aTi, bTi_all, sims);
  Values result = aligner.solve();

  const auto recoveredSim = result.at<Similarity3>(Symbol('S', 0));
  EXPECT(simClose(gtSim1, recoveredSim, 2e-2));

  for (const auto& kv : parent) {
    const Key key = kv.key;
    EXPECT(assert_equal<Pose3>(parent.at<Pose3>(key), result.at<Pose3>(key), 1e-2));
  }
}

/* ************************************************************************* */
TEST(TrajectoryAlignerSim3, SingleChildWithExtraNonOverlap) {
  const auto parent = makeParentValues();
  auto child = transformValues(gtSim1, parent);
  const auto perturbedChild = perturbPoses(child);

  PoseMeasurements aTi = makeMeasurements(parent, /*noiseSigma=*/1e-2);
  PoseMeasurements bTi = makeMeasurements(perturbedChild, 1e-1);
  // Add a non-overlapping camera in the child frame.
  bTi.emplace_back(10,
                   gtSim1.transformFrom(Pose3(Rot3(), Point3(4.0, -1.0, 0.5))),
                   noiseModel::Isotropic::Sigma(6, 1e-2));

  ChildrenPoses bTi_all{bTi};
  std::vector<Similarity3> sims{perturbSim3(gtSim1)};

  TrajectoryAlignerSim3 aligner(aTi, bTi_all, sims);
  Values result = aligner.solve();

  const auto recoveredSim = result.at<Similarity3>(Symbol('S', 0));
  EXPECT(simClose(gtSim1, recoveredSim, 2e-2));

  for (const auto& kv : parent) {
    const Key key = kv.key;
    EXPECT(assert_equal<Pose3>(parent.at<Pose3>(key), result.at<Pose3>(key), 1e-2));
  }
}

/* ************************************************************************* */
TEST(TrajectoryAlignerSim3, TwoChildrenNoisy) {
  const auto parent = makeParentValues();

  PoseMeasurements aTi = makeMeasurements(parent, 1e-2);
  PoseMeasurements b1 =
      makeMeasurements(perturbPoses(transformValues(gtSim1, parent)), 5e-2);
  PoseMeasurements b2 =
      makeMeasurements(perturbPoses(transformValues(gtSim2, parent)), 5e-2);

  ChildrenPoses bTi_all{b1, b2};
  std::vector<Similarity3> sims{perturbSim3(gtSim1), perturbSim3(gtSim2)};

  TrajectoryAlignerSim3 aligner(aTi, bTi_all, sims);
  Values result = aligner.solve();

  EXPECT(simClose(gtSim1, result.at<Similarity3>(Symbol('S', 0)), 5e-2));
  EXPECT(simClose(gtSim2, result.at<Similarity3>(Symbol('S', 1)), 5e-2));

  for (const auto& kv : parent) {
    const Key key = kv.key;
    EXPECT(assert_equal<Pose3>(parent.at<Pose3>(key), result.at<Pose3>(key), 1e-2));
  }
}

/* ************************************************************************* */
TEST(TrajectoryAlignerSim3, ThreeChildrenNoisy) {
  const auto parent = makeParentValues();

  PoseMeasurements aTi = makeMeasurements(parent, 1e-2);
  PoseMeasurements b1 =
      makeMeasurements(perturbPoses(transformValues(gtSim1, parent)), 2e-2);
  PoseMeasurements b2 =
      makeMeasurements(perturbPoses(transformValues(gtSim2, parent)), 2e-2);
  PoseMeasurements b3 =
      makeMeasurements(perturbPoses(transformValues(gtSim3, parent)), 2e-2);

  ChildrenPoses bTi_all{b1, b2, b3};
  std::vector<Similarity3> sims{perturbSim3(gtSim1), perturbSim3(gtSim2),
                                perturbSim3(gtSim3)};

  TrajectoryAlignerSim3 aligner(aTi, bTi_all, sims);
  Values result = aligner.solve();

  EXPECT(simClose(gtSim1, result.at<Similarity3>(Symbol('S', 0)), 1e-1));
  EXPECT(simClose(gtSim2, result.at<Similarity3>(Symbol('S', 1)), 1e-1));
  EXPECT(simClose(gtSim3, result.at<Similarity3>(Symbol('S', 2)), 1e-1));

  for (const auto& kv : parent) {
    const Key key = kv.key;
    EXPECT(assert_equal<Pose3>(parent.at<Pose3>(key), result.at<Pose3>(key), 1e-2));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
