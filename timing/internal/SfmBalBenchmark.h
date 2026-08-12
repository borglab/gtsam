/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmBalBenchmark.h
 * @brief Private shared infrastructure for BAL timing executables.
 */

#pragma once

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/GeneralSFMFactor.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace gtsam::timing::bal {

using Camera = PinholeCamera<Cal3Bundler>;
using SfmFactor = GeneralSFMFactor<Camera, Point3>;

/** Explicit configuration shared by BAL graph builders and solvers. */
struct BalBenchmarkConfig {
  bool useSchur = true;
  SharedNoiseModel projectionNoise = noiseModel::Unit::Create(2);
};

/** Result of parsing the common small-BAL executable command line. */
struct BalBenchmarkInput {
  BalBenchmarkConfig config;
  SfmData data;
  std::string filename;
};

/// Return the default 16-camera BAL dataset path.
std::string defaultDataset();

/// Return the default 135-camera profiling BAL dataset path.
std::string profileDataset();

/// Return the standard 16-, 88-, and 135-camera benchmark dataset paths.
std::vector<std::string> standardDatasets();

/** Load one BAL dataset from a selected filename. */
SfmData loadDataset(const std::string& filename);

/** Parse `[--colamd] [BALfile]` and load the selected dataset. */
BalBenchmarkInput parseSmallBenchmark(int argc, char* argv[]);

/** Build the conventional GeneralSFMFactor graph for a BAL dataset. */
NonlinearFactorGraph buildGeneralSfmGraph(
    const SfmData& data, const BalBenchmarkConfig& config,
    std::optional<double> priorStddev = std::nullopt);

/** Build conventional camera and landmark initial values. */
Values buildGeneralSfmInitial(const SfmData& data);

/** Create a point-first Schur ordering, optionally separating calibration. */
Ordering createSchurOrdering(const SfmData& data, bool separateCalibration);

/** Create the ordering variants used by multifrontal timing programs. */
std::vector<std::pair<std::string, Ordering>> createOrderings(
    const SfmData& data, const GaussianFactorGraph& linear);

/** Build point-batch factors, optionally chunking long tracks. */
NonlinearFactorGraph buildBatchSfmGraph(const SfmData& data,
                                        const BalBenchmarkConfig& config,
                                        bool useHessianFactor,
                                        size_t chunkSize);

/** Build camera-batch factors for the camera-first comparison. */
NonlinearFactorGraph buildCameraBatchSfmGraph(const SfmData& data,
                                              const BalBenchmarkConfig& config);

/** Create the camera-first ordering used with camera-batch factors. */
Ordering createCameraFirstOrdering(const SfmData& data);

/** Return BAL LM defaults aligned with Ceres and existing timing programs. */
LevenbergMarquardtParams makeLevenbergMarquardtParams(
    const BalBenchmarkConfig& config, const Ordering* ordering = nullptr,
    const std::string& verbosity = "SUMMARY");

/** Run the legacy small-BAL optimizer and print gttic timing output. */
int optimize(const SfmData& data, const NonlinearFactorGraph& graph,
             const Values& initial, const BalBenchmarkConfig& config,
             bool separateCalibration = false, bool useMetisOrdering = false);

}  // namespace gtsam::timing::bal
