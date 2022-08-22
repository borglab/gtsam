/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartStereoFactor_iSAM2.cpp
 *  @brief Unit tests for ProjectionFactor Class
 *  @author Jose Luis Blanco-Claraco
 *  @date   May 2019
 *
 *  @note Originally based on ISAM2_SmartFactorStereo.cpp by Nghia Ho
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/debug.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// Set to 1 to enable verbose output of intermediary results
#define TEST_VERBOSE_OUTPUT 0

#if TEST_VERBOSE_OUTPUT
#define TEST_COUT(ARGS_) std::cout << ARGS_
#else
#define TEST_COUT(ARGS_) void(0)
#endif

// Tolerance for ground-truth pose comparison:
static const double tol = 1e-3;

// Synthetic dataset generated with rwt
// (https://github.com/jlblancoc/recursive-world-toolkit)
// Camera parameters
const double fx = 200.0;
const double fy = 150.0;
const double cx = 512.0;
const double cy = 384.0;
const double baseline = 0.2; // meters

using timestep_t = std::size_t;
using lm_id_t = int;

struct stereo_meas_t {
  stereo_meas_t(lm_id_t id, double lu, double ru, double v_lr)
      : lm_id{id}, left_u{lu}, right_u{ru}, v{v_lr} {}

  lm_id_t lm_id{-1}; // landmark id
  double left_u{0}, right_u{0}, v{0};
};

static std::map<timestep_t, std::vector<stereo_meas_t>> dataset = {
    {0,
     {{0, 911.99993896, 712.00000000, 384.0},
      {159, 311.99996948, 211.99996948, 384.0},
      {3, 378.66665649, 312.00000000, 384.0},
      {2, 645.33331299, 578.66662598, 384.0},
      {157, 111.99994659, 11.99993896, 384.0},
      {4, 578.66662598, 545.33331299, 384.0},
      {5, 445.33331299, 412.00000000, 384.0},
      {6, 562.00000000, 537.00000000, 384.0}}},
    {1,
     {{0, 1022.06353760, 762.57519531, 384.0},
      {159, 288.30487061, 177.80273438, 384.0},
      {2, 655.30645752, 583.12127686, 384.0},
      {3, 368.60937500, 297.43176270, 384.0},
      {4, 581.82666016, 547.16766357, 384.0},
      {5, 443.66183472, 409.23681641, 384.0},
      {6, 564.35980225, 538.62115479, 384.0},
      {7, 461.66418457, 436.05477905, 384.0},
      {8, 550.32220459, 531.75256348, 384.0},
      {9, 476.17767334, 457.67541504, 384.0}}},
    {2,
     {{159, 257.97128296, 134.26287842, 384.0},
      {2, 666.87255859, 588.07275391, 384.0},
      {3, 356.53823853, 280.10061646, 384.0},
      {4, 585.10949707, 548.99212646, 384.0},
      {5, 441.66403198, 406.05108643, 384.0},
      {6, 566.75402832, 540.21868896, 384.0},
      {7, 461.16207886, 434.90002441, 384.0},
      {8, 552.28387451, 533.30230713, 384.0},
      {9, 476.63549805, 457.79418945, 384.0},
      {10, 546.48394775, 530.53009033, 384.0}}},
    {3,
     {{159, 218.10592651, 77.30914307, 384.0},
      {2, 680.54644775, 593.68103027, 384.0},
      {3, 341.92507935, 259.28231812, 384.0},
      {4, 588.53289795, 550.80499268, 384.0},
      {5, 439.29989624, 402.39105225, 384.0},
      {6, 569.18627930, 541.78991699, 384.0},
      {7, 460.47863770, 433.51678467, 384.0},
      {8, 554.24902344, 534.82952881, 384.0},
      {9, 477.00451660, 457.80438232, 384.0},
      {10, 548.33770752, 532.07501221, 384.0},
      {11, 483.58688354, 467.47830200, 384.0},
      {12, 542.36785889, 529.29321289, 384.0}}},
    {4,
     {{2, 697.09454346, 600.18432617, 384.0},
      {3, 324.03643799, 233.97094727, 384.0},
      {4, 592.11877441, 552.60449219, 384.0},
      {5, 436.52197266, 398.19531250, 384.0},
      {6, 571.66101074, 543.33209229, 384.0},
      {7, 459.59658813, 431.88333130, 384.0},
      {8, 556.21801758, 536.33258057, 384.0},
      {9, 477.27893066, 457.69882202, 384.0},
      {10, 550.18920898, 533.60003662, 384.0},
      {11, 484.24472046, 467.86862183, 384.0},
      {12, 544.14727783, 530.86157227, 384.0},
      {13, 491.26141357, 478.11267090, 384.0},
      {14, 541.29949951, 529.57086182, 384.0},
      {15, 494.58111572, 482.95935059, 384.0}}}};

// clang-format off
/*
% Ground truth path of the SENSOR, and the ROBOT
% STEP     X       Y        Z        QR        QX      QY      QZ     |    X Y Z        QR        QX      QY      QZ
 ----------------------------------------------------------------------------------------------------------------------------------------
     0 0.000000  0.000000 0.000000 0.500000 -0.500000 0.500000 -0.500000 0.000000  0.000000 0.000000 1.000000 0.000000 0.000000 0.000000
     1 0.042019 -0.008403 0.000000 0.502446 -0.502446 0.497542 -0.497542 0.042019 -0.008403 0.000000 0.999988 0.000000 0.000000 0.004905
     2 0.084783 -0.016953 0.000000 0.504879 -0.504879 0.495073 -0.495073 0.084783 -0.016953 0.000000 0.999952 0.000000 0.000000 0.009806
     3 0.128305 -0.025648 0.000000 0.507299 -0.507299 0.492592 -0.492592 0.128305 -0.025648 0.000000 0.999892 0.000000 0.000000 0.014707
     4 0.172605 -0.034490 0.000000 0.509709 -0.509709 0.490098 -0.490098 0.172605 -0.034490 0.000000 0.999808 0.000000 0.000000 0.019611
*/
// clang-format on

// Ground truth using camera pose = vehicle frame
// The table above uses:
//   camera +x = vehicle -y
//   camera +y = vehicle -z
//   camera +z = vehicle +x
static const std::map<timestep_t, gtsam::Point3> gt_positions = {
    {0, {0.000000, 0.000000, 0.0}},
    {1, {0.042019, -0.008403, 0.0}},
    {2, {0.084783, -0.016953, 0.0}},
    {3, {0.128305, -0.025648, 0.0}},
    {4, {0.172605, -0.034490, 0.0}}};

// Batch version, to compare against iSAM2 solution.
TEST(testISAM2SmartFactor, Stereo_Batch) {
  TEST_COUT("============ Running: Batch ============\n");

  using namespace gtsam;
  using symbol_shorthand::V;
  using symbol_shorthand::X;

  const auto K =
      boost::make_shared<Cal3_S2Stereo>(fx, fy, .0, cx, cy, baseline);

  // Pose prior - at identity
  auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.2), Vector3::Constant(0.2)).finished());

  // Map: landmark_id => smart_factor_index inside iSAM2
  std::map<lm_id_t, FactorIndex> lm2factor;

  // Storage of smart factors:
  std::map<lm_id_t, SmartStereoProjectionPoseFactor::shared_ptr> smartFactors;

  NonlinearFactorGraph batch_graph;
  Values batch_values;

  // Run one timestep at once:
  for (const auto &entries : dataset) {
    // 1) Process new observations:
    // ------------------------------
    const auto kf_id = entries.first;
    const std::vector<stereo_meas_t> &obs = entries.second;

    for (const stereo_meas_t &stObs : obs) {
      if (smartFactors.count(stObs.lm_id) == 0) {
        auto noise = noiseModel::Isotropic::Sigma(3, 0.1);
        SmartProjectionParams parm(HESSIAN, ZERO_ON_DEGENERACY);

        smartFactors[stObs.lm_id] =
            boost::make_shared<SmartStereoProjectionPoseFactor>(noise, parm);

        batch_graph.push_back(smartFactors[stObs.lm_id]);
      }

      TEST_COUT("Adding stereo observation from KF #" << kf_id << " for LM #"
                                                      << stObs.lm_id << "\n");

      smartFactors[stObs.lm_id]->add(
          StereoPoint2(stObs.left_u, stObs.right_u, stObs.v), X(kf_id), K);
    }

    // prior, for the first keyframe:
    if (kf_id == 0) {
      batch_graph.addPrior(X(kf_id), Pose3::Identity(), priorPoseNoise);
    }

    batch_values.insert(X(kf_id), Pose3::Identity());
  }

  LevenbergMarquardtParams parameters;
#if TEST_VERBOSE_OUTPUT
  parameters.verbosity = NonlinearOptimizerParams::LINEAR;
  parameters.verbosityLM = LevenbergMarquardtParams::TRYDELTA;
#endif

  LevenbergMarquardtOptimizer lm(batch_graph, batch_values, parameters);

  Values finalEstimate = lm.optimize();
#if TEST_VERBOSE_OUTPUT
  finalEstimate.print("LevMarq estimate:");
#endif

  // GT:
  //   camera +x = vehicle -y
  //   camera +y = vehicle -z
  //   camera +z = vehicle +x
  for (const auto &gt : gt_positions) {
    const Pose3 p = finalEstimate.at<Pose3>(X(gt.first));
    EXPECT(assert_equal(p.x(), -gt.second.y(), tol));
    EXPECT(assert_equal(p.y(), -gt.second.z(), tol));
    EXPECT(assert_equal(p.z(), gt.second.x(), tol));
  }
}

TEST(testISAM2SmartFactor, Stereo_iSAM2) {
  TEST_COUT("======= Running: iSAM2 ==========\n");

#if TEST_VERBOSE_OUTPUT
  SETDEBUG("ISAM2 update", true);
  // SETDEBUG("ISAM2 update verbose",true);
#endif

  using namespace gtsam;
  using symbol_shorthand::V;
  using symbol_shorthand::X;

  const auto K =
      boost::make_shared<Cal3_S2Stereo>(fx, fy, .0, cx, cy, baseline);

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.evaluateNonlinearError = true;

  // Do not cache smart factors:
  parameters.cacheLinearizedFactors = false;

  // Important: must set relinearizeSkip=1 to additional calls to update() to
  // have a real effect.
  parameters.relinearizeSkip = 1;

  ISAM2 isam(parameters);

  // Pose prior - at identity
  auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.2), Vector3::Constant(0.2)).finished());

  // Map: landmark_id => smart_factor_index inside iSAM2
  std::map<lm_id_t, FactorIndex> lm2factor;

  // Storage of smart factors:
  std::map<lm_id_t, SmartStereoProjectionPoseFactor::shared_ptr> smartFactors;

  Pose3 lastKeyframePose = Pose3::Identity();

  // Run one timestep at once:
  for (const auto &entries : dataset) {
    // 1) Process new observations:
    // ------------------------------
    const auto kf_id = entries.first;
    const std::vector<stereo_meas_t> &obs = entries.second;

    // Special instructions for using iSAM2 + smart factors:
    // Must fill factorNewAffectedKeys:
    FastMap<FactorIndex, KeySet> factorNewAffectedKeys;
    NonlinearFactorGraph newFactors;

    // Map: factor index in the internal graph of iSAM2 => landmark_id
    std::map<FactorIndex, lm_id_t> newFactor2lm;

    for (const stereo_meas_t &stObs : obs) {
      if (smartFactors.count(stObs.lm_id) == 0) {
        auto noise = noiseModel::Isotropic::Sigma(3, 0.1);
        SmartProjectionParams params(HESSIAN, ZERO_ON_DEGENERACY);

        smartFactors[stObs.lm_id] =
            boost::make_shared<SmartStereoProjectionPoseFactor>(noise, params);
        newFactor2lm[newFactors.size()] = stObs.lm_id;
        newFactors.push_back(smartFactors[stObs.lm_id]);
      } else {
        // Only if the factor *already* existed:
        factorNewAffectedKeys[lm2factor.at(stObs.lm_id)].insert(X(kf_id));
      }

      TEST_COUT("Adding stereo observation from KF #" << kf_id << " for LM #"
                                                      << stObs.lm_id << "\n");

      smartFactors[stObs.lm_id]->add(
          StereoPoint2(stObs.left_u, stObs.right_u, stObs.v), X(kf_id), K);
    }

    // prior, for the first keyframe:
    if (kf_id == 0) {
      newFactors.addPrior(X(kf_id), Pose3::Identity(), priorPoseNoise);
    }

    // 2) Run iSAM2:
    // ------------------------------
    Values newValues;
    newValues.insert(X(kf_id), lastKeyframePose);

    TEST_COUT("Running iSAM2 for frame: " << kf_id << "\n");

    ISAM2UpdateParams updateParams;
    updateParams.newAffectedKeys = std::move(factorNewAffectedKeys);

    ISAM2Result res = isam.update(newFactors, newValues, updateParams);

    for (const auto &f2l : newFactor2lm)
      lm2factor[f2l.second] = res.newFactorsIndices.at(f2l.first);

    TEST_COUT("error before1: " << res.errorBefore.value() << "\n");
    TEST_COUT("error after1 : " << res.errorAfter.value() << "\n");

    // Additional refining steps:
    ISAM2Result res2 = isam.update();
    TEST_COUT("error before2: " << res2.errorBefore.value() << "\n");
    TEST_COUT("error after2 : " << res2.errorAfter.value() << "\n");

    Values currentEstimate = isam.calculateEstimate();
#if TEST_VERBOSE_OUTPUT
    currentEstimate.print("currentEstimate:");
#endif

    // Keep last KF pose as initial pose of the next one, to reduce the need
    // to run more non-linear iterations:
    lastKeyframePose = currentEstimate.at(X(kf_id)).cast<Pose3>();

  } // end for each timestep

  Values finalEstimate = isam.calculateEstimate();

  // GT:
  //   camera +x = vehicle -y
  //   camera +y = vehicle -z
  //   camera +z = vehicle +x
  for (const auto &gt : gt_positions) {
    const Pose3 p = finalEstimate.at<Pose3>(X(gt.first));
    EXPECT(assert_equal(p.x(), -gt.second.y(), tol));
    EXPECT(assert_equal(p.y(), -gt.second.z(), tol));
    EXPECT(assert_equal(p.z(), gt.second.x(), tol));
  }
}

/* *************************************************************************
 */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
