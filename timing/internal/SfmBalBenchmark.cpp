/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmBalBenchmark.cpp
 * @brief Shared BAL dataset, graph, ordering, and optimizer support.
 */

#include "SfmBalBenchmark.h"

#include <gtsam/base/timing.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/SmartProjectionFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <map>
#include <stdexcept>

#include "TimingUtils.h"

namespace gtsam::timing::bal {
namespace {

using symbol_shorthand::C;
using symbol_shorthand::K;
using symbol_shorthand::P;

constexpr const char* kDefaultDataset = "dubrovnik-16-22106-pre";
constexpr const char* kProfileDataset = "dubrovnik-135-90642-pre";

}  // namespace

std::string defaultDataset() { return findExampleDataFile(kDefaultDataset); }

std::string profileDataset() { return findExampleDataFile(kProfileDataset); }

std::vector<std::string> standardDatasets() {
  return {defaultDataset(), findExampleDataFile("dubrovnik-88-64298-pre"),
          profileDataset()};
}

SfmData loadDataset(const std::string& filename) {
  return SfmData::FromBalFile(filename);
}

BalBenchmarkInput parseSmallBenchmark(int argc, char* argv[]) {
  Arguments arguments(argc, argv);
  BalBenchmarkInput input;
  input.config.useSchur = !arguments.flag("--colamd");
  const std::vector<std::string> filenames = arguments.positionals();
  arguments.validateAllConsumed();
  if (filenames.size() > 1) {
    throw std::runtime_error("Usage: timeSFMBALxxx [--colamd] [BALfile]");
  }
  input.filename = filenames.empty() ? defaultDataset() : filenames.front();
  input.data = loadDataset(input.filename);
  return input;
}

NonlinearFactorGraph buildGeneralSfmGraph(const SfmData& data,
                                          const BalBenchmarkConfig& config,
                                          std::optional<double> priorStddev) {
  NonlinearFactorGraph graph;
  std::vector<bool> hasCameraMeasurement(data.numberCameras(), false);
  for (size_t trackIndex = 0; trackIndex < data.numberTracks(); ++trackIndex) {
    const auto& measurements = data.tracks[trackIndex].measurements;
    if (measurements.size() < 2) continue;
    for (const SfmMeasurement& measurement : measurements) {
      const size_t cameraIndex = measurement.first;
      graph.emplace_shared<SfmFactor>(measurement.second,
                                      config.projectionNoise, C(cameraIndex),
                                      P(trackIndex));
      hasCameraMeasurement[cameraIndex] = true;
    }
  }

  if (priorStddev) {
    const auto cameraPriorNoise = noiseModel::Isotropic::Sigma(9, *priorStddev);
    for (size_t i = 0; i < hasCameraMeasurement.size(); ++i) {
      if (hasCameraMeasurement[i]) {
        graph.addPrior<Camera>(C(i), data.cameras[i], cameraPriorNoise);
      }
    }

    const auto pointPriorNoise = noiseModel::Isotropic::Sigma(3, *priorStddev);
    for (size_t j = 0; j < data.numberTracks(); ++j) {
      graph.addPrior<Point3>(P(j), data.tracks[j].p, pointPriorNoise);
    }
  }
  return graph;
}

Values buildGeneralSfmInitial(const SfmData& data) {
  Values initial;
  for (size_t i = 0; i < data.cameras.size(); ++i) {
    initial.insert(C(i), data.cameras[i]);
  }
  for (size_t j = 0; j < data.tracks.size(); ++j) {
    initial.insert(P(j), data.tracks[j].p);
  }
  return initial;
}

NonlinearFactorGraph buildSmartSfmGraph(
    const SfmData& data, const BalBenchmarkConfig& config,
    const SmartProjectionParams& smartParams) {
  using SmartSfmFactor = SmartProjectionFactor<Camera>;

  NonlinearFactorGraph graph;
  for (const SfmTrack& track : data.tracks) {
    if (track.measurements.size() < 2) continue;
    auto factor = std::make_shared<SmartSfmFactor>(config.projectionNoise,
                                                   smartParams);
    for (const SfmMeasurement& measurement : track.measurements) {
      factor->add(measurement.second, C(measurement.first));
    }
    graph.push_back(factor);
  }
  return graph;
}

Values buildSmartSfmInitial(const SfmData& data) {
  Values initial;
  for (size_t i = 0; i < data.cameras.size(); ++i) {
    initial.insert(C(i), data.cameras[i]);
  }
  return initial;
}

Ordering createCameraOrdering(const SfmData& data) {
  Ordering ordering;
  for (size_t i = 0; i < data.numberCameras(); ++i) ordering.push_back(C(i));
  return ordering;
}

Ordering createSchurOrdering(const SfmData& data, bool separateCalibration) {
  Ordering ordering;
  for (size_t j = 0; j < data.numberTracks(); ++j) ordering.push_back(P(j));
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    ordering.push_back(C(i));
    if (separateCalibration) ordering.push_back(K(i));
  }
  return ordering;
}

std::vector<std::pair<std::string, Ordering>> createOrderings(
    const SfmData& data, const GaussianFactorGraph& linear) {
  return {{"Burn", createSchurOrdering(data, false)},
          {"Metis", Ordering::Metis(linear)},
          {"Schur", createSchurOrdering(data, false)},
          {"Colamd", Ordering::Colamd(linear)}};
}

NonlinearFactorGraph buildBatchSfmGraph(const SfmData& data,
                                        const BalBenchmarkConfig& config,
                                        bool useHessianFactor,
                                        size_t chunkSize) {
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < data.numberTracks(); ++j) {
    const auto& trackMeasurements = data.tracks[j].measurements;
    if (trackMeasurements.size() < 2) continue;

    const size_t measurementCount = trackMeasurements.size();
    const size_t effectiveChunkSize =
        chunkSize == 0 ? measurementCount
                       : std::min(chunkSize, measurementCount);
    for (size_t start = 0; start < measurementCount;
         start += effectiveChunkSize) {
      const size_t end = std::min(start + effectiveChunkSize, measurementCount);
      std::map<Key, Point2> measurements;
      for (size_t i = start; i < end; ++i) {
        const SfmMeasurement& measurement = trackMeasurements[i];
        measurements[C(measurement.first)] = measurement.second;
      }
      auto batch = std::make_shared<BatchFactor<SfmFactor, 2>>(
          measurements, P(j), config.projectionNoise);
      batch->setUseHessianFactor(useHessianFactor);
      graph.add(batch);
    }
  }
  return graph;
}

NonlinearFactorGraph buildCameraBatchSfmGraph(
    const SfmData& data, const BalBenchmarkConfig& config) {
  NonlinearFactorGraph graph;
  std::vector<std::map<Key, Point2>> measurementsByCamera(data.numberCameras());
  for (size_t j = 0; j < data.numberTracks(); ++j) {
    const auto& trackMeasurements = data.tracks[j].measurements;
    if (trackMeasurements.size() < 2) continue;
    for (const SfmMeasurement& measurement : trackMeasurements) {
      measurementsByCamera[measurement.first][P(j)] = measurement.second;
    }
  }

  for (size_t i = 0; i < measurementsByCamera.size(); ++i) {
    if (measurementsByCamera[i].empty()) continue;
    graph.add(std::make_shared<BatchFactor<SfmFactor, 2>>(
        C(i), measurementsByCamera[i], config.projectionNoise));
  }
  return graph;
}

Ordering createCameraFirstOrdering(const SfmData& data) {
  Ordering ordering;
  for (size_t i = 0; i < data.numberCameras(); ++i) ordering.push_back(C(i));
  for (size_t j = 0; j < data.numberTracks(); ++j) ordering.push_back(P(j));
  return ordering;
}

LevenbergMarquardtParams makeLevenbergMarquardtParams(
    const BalBenchmarkConfig& config, const Ordering* ordering,
    const std::string& verbosity) {
  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.setVerbosityLM(verbosity);
  params.setRelativeErrorTol(0.01);
  if (config.useSchur && ordering) params.setOrdering(*ordering);
  return params;
}

int optimize(const SfmData& data, const NonlinearFactorGraph& graph,
             const Values& initial, const BalBenchmarkConfig& config,
             bool separateCalibration, bool useMetisOrdering) {
  LevenbergMarquardtParams params = makeLevenbergMarquardtParams(config);
  if (useMetisOrdering) {
    params.setOrderingType("METIS");
  } else if (config.useSchur) {
    params.setOrdering(createSchurOrdering(data, separateCalibration));
  }

  {
    gttic_(optimize);
    LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    const Values result = optimizer.optimize();
    (void)result;
  }
  tictoc_finishedIteration_();
  tictoc_print_();
  return 0;
}

}  // namespace gtsam::timing::bal
