/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmProjectionBatch.h
 * @brief   Device-resident batch of SFM projection measurements
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/cuda/SfmTypes.h>

#include <cuda_runtime_api.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {

enum class SfmProjectionNoiseMode {
  Unit,
  Whitened,
  Robust,
};

struct SfmProjectionBatchTransferProfile {
  double hostBuildElapsed = 0.0;
  double deviceAllocElapsed = 0.0;
  DeviceTransferSummary h2d;
};

class SfmProjectionBatch {
 public:
  static constexpr int kLongTrackMeasurementThreshold = 4;

  static SfmProjectionBatch fromSfmData(
      const SfmData& data, cudaStream_t stream = nullptr,
      SfmProjectionBatchTransferProfile* profile = nullptr) {
    return fromSfmDataImpl(data, nullptr, nullptr, stream, profile);
  }

  static SfmProjectionBatch fromSfmData(
      const SfmData& data,
      const std::vector<std::vector<SfmSqrtInfo2>>& sqrtInfoByTrack,
      cudaStream_t stream = nullptr,
      SfmProjectionBatchTransferProfile* profile = nullptr) {
    return fromSfmDataImpl(data, &sqrtInfoByTrack, nullptr, stream, profile);
  }

  static SfmProjectionBatch fromSfmData(
      const SfmData& data,
      const std::vector<std::vector<SfmSqrtInfo2>>& sqrtInfoByTrack,
      const std::vector<std::vector<SfmRobustModel>>& robustModelsByTrack,
      cudaStream_t stream = nullptr,
      SfmProjectionBatchTransferProfile* profile = nullptr) {
    return fromSfmDataImpl(data, &sqrtInfoByTrack, &robustModelsByTrack,
                           stream, profile);
  }

  static SfmSqrtInfo2 unitSqrtInfo() {
    return SfmSqrtInfo2{1.0, 0.0, 1.0};
  }

  static SfmRobustModel noRobustModel() {
    return SfmRobustModel{SfmRobustModelKind::None,
                          SfmRobustReweightScheme::Block, 0.0};
  }

  static bool isUnitSqrtInfo(const SfmSqrtInfo2& sqrtInfo) {
    return sqrtInfo.r00 == 1.0 && sqrtInfo.r01 == 0.0 &&
           sqrtInfo.r11 == 1.0;
  }

  // Zero diagonals are allowed: they encode a zero-information (fully
  // down-weighted) measurement, as produced by GNC weighted graphs.
  static bool isUsableSqrtInfo(const SfmSqrtInfo2& sqrtInfo) {
    return std::isfinite(sqrtInfo.r00) && std::isfinite(sqrtInfo.r01) &&
           std::isfinite(sqrtInfo.r11) && sqrtInfo.r00 >= 0.0 &&
           sqrtInfo.r11 >= 0.0;
  }

  static bool isUsableRobustModel(const SfmRobustModel& robustModel) {
    if (robustModel.kind == SfmRobustModelKind::None) {
      return robustModel.reweightScheme == SfmRobustReweightScheme::Block;
    }
    const bool knownModel =
        robustModel.kind == SfmRobustModelKind::Huber ||
        robustModel.kind == SfmRobustModelKind::Tukey;
    const bool knownReweight =
        robustModel.reweightScheme == SfmRobustReweightScheme::Scalar ||
        robustModel.reweightScheme == SfmRobustReweightScheme::Block;
    return knownModel && knownReweight && std::isfinite(robustModel.parameter) &&
           robustModel.parameter > 0.0;
  }

 private:
  static SfmProjectionBatch fromSfmDataImpl(
      const SfmData& data,
      const std::vector<std::vector<SfmSqrtInfo2>>* sqrtInfoByTrack,
      const std::vector<std::vector<SfmRobustModel>>* robustModelsByTrack,
      cudaStream_t stream,
      SfmProjectionBatchTransferProfile* profile) {
    if (profile) {
      *profile = SfmProjectionBatchTransferProfile{};
    }
    const auto hostBuildStart =
        profile ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point{};
    SfmProjectionBatch batch;
    batch.numCameras_ = data.numberCameras();
    batch.numPoints_ = data.numberTracks();
    batch.noiseMode_ =
        robustModelsByTrack ? SfmProjectionNoiseMode::Robust
        : sqrtInfoByTrack  ? SfmProjectionNoiseMode::Whitened
                           : SfmProjectionNoiseMode::Unit;

    if (sqrtInfoByTrack && sqrtInfoByTrack->size() != data.numberTracks()) {
      throw std::invalid_argument(
          "SfmProjectionBatch sqrt-info track count mismatch");
    }
    if (robustModelsByTrack) {
      if (!sqrtInfoByTrack) {
        throw std::invalid_argument(
            "SfmProjectionBatch robust models require sqrt-info");
      }
      if (robustModelsByTrack->size() != data.numberTracks()) {
        throw std::invalid_argument(
            "SfmProjectionBatch robust model track count mismatch");
      }
    }

    std::vector<SfmObservation> observations;
    std::vector<SfmSqrtInfo2> sqrtInfos;
    std::vector<SfmRobustModel> robustModels;
    std::vector<int> pointObservationOffsets;
    std::vector<int> longTrackPointSlots;

    // Size the observation arrays from the track measurement counts. Reading
    // only the counts stays inside the contiguous track array, so this pass
    // does not touch the scattered measurement storage, and it turns the fill
    // below into a straight append instead of ~20 reallocate-and-copy rounds
    // over what reaches tens of megabytes on a large BAL problem.
    size_t totalMeasurements = 0;
    for (size_t pointSlot = 0; pointSlot < data.numberTracks(); ++pointSlot) {
      totalMeasurements += data.track(pointSlot).numberMeasurements();
    }
    observations.reserve(totalMeasurements);
    if (sqrtInfoByTrack) sqrtInfos.reserve(totalMeasurements);
    if (robustModelsByTrack) robustModels.reserve(totalMeasurements);
    pointObservationOffsets.reserve(data.numberTracks() + 1);
    pointObservationOffsets.push_back(0);
    for (size_t pointSlot = 0; pointSlot < data.numberTracks(); ++pointSlot) {
      if (pointSlot > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument(
            "SfmProjectionBatch point index too large");
      }

      const SfmTrack& track = data.track(pointSlot);
      if (sqrtInfoByTrack &&
          (*sqrtInfoByTrack)[pointSlot].size() !=
              track.numberMeasurements()) {
        throw std::invalid_argument(
            "SfmProjectionBatch sqrt-info measurement count mismatch");
      }
      if (robustModelsByTrack &&
          (*robustModelsByTrack)[pointSlot].size() !=
              track.numberMeasurements()) {
        throw std::invalid_argument(
            "SfmProjectionBatch robust model measurement count mismatch");
      }
      if (track.numberMeasurements() < 2) {
        pointObservationOffsets.push_back(
            static_cast<int>(observations.size()));
        continue;
      }

      if (track.numberMeasurements() > kLongTrackMeasurementThreshold) {
        longTrackPointSlots.push_back(static_cast<int>(pointSlot));
      }

      for (size_t measurementSlot = 0;
           measurementSlot < track.measurements.size(); ++measurementSlot) {
        const SfmMeasurement& measurement = track.measurements[measurementSlot];
        if (measurement.first >= data.numberCameras() ||
            measurement.first >
                static_cast<size_t>(std::numeric_limits<int>::max())) {
          throw std::invalid_argument(
              "SfmProjectionBatch camera index out of range");
        }

        observations.push_back(
            {static_cast<int>(measurement.first), static_cast<int>(pointSlot),
             measurement.second(0), measurement.second(1)});
        if (sqrtInfoByTrack) {
          const SfmSqrtInfo2 sqrtInfo =
              (*sqrtInfoByTrack)[pointSlot][measurementSlot];
          if (!isUsableSqrtInfo(sqrtInfo)) {
            throw std::invalid_argument(
                "SfmProjectionBatch sqrt-info contains invalid values");
          }
          sqrtInfos.push_back(sqrtInfo);
        }
        if (robustModelsByTrack) {
          const SfmRobustModel robustModel =
              (*robustModelsByTrack)[pointSlot][measurementSlot];
          if (!isUsableRobustModel(robustModel)) {
            throw std::invalid_argument(
                "SfmProjectionBatch robust model contains invalid values");
          }
          robustModels.push_back(robustModel);
        }
      }
      pointObservationOffsets.push_back(static_cast<int>(observations.size()));
    }

    if (profile) {
      profile->hostBuildElapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        hostBuildStart)
              .count();
      profile->h2d.add(batch.observations_.uploadProfiled(observations,
                                                          stream));
      profile->h2d.add(batch.pointObservationOffsets_.uploadProfiled(
          pointObservationOffsets, stream));
      profile->h2d.add(batch.longTrackPointSlots_.uploadProfiled(
          longTrackPointSlots, stream));
    } else {
      batch.observations_.upload(observations, stream);
      batch.pointObservationOffsets_.upload(pointObservationOffsets, stream);
      batch.longTrackPointSlots_.upload(longTrackPointSlots, stream);
    }
    if (sqrtInfoByTrack) {
      if (sqrtInfos.size() != observations.size()) {
        throw std::invalid_argument(
            "SfmProjectionBatch sqrt-info/observation count mismatch");
      }
      if (profile) {
        profile->h2d.add(batch.sqrtInfos_.uploadProfiled(sqrtInfos, stream));
      } else {
        batch.sqrtInfos_.upload(sqrtInfos, stream);
      }
    }
    if (robustModelsByTrack) {
      if (robustModels.size() != observations.size()) {
        throw std::invalid_argument(
            "SfmProjectionBatch robust model/observation count mismatch");
      }
      if (profile) {
        profile->h2d.add(
            batch.robustModels_.uploadProfiled(robustModels, stream));
      } else {
        batch.robustModels_.upload(robustModels, stream);
      }
    }
    if (profile) {
      profile->deviceAllocElapsed = profile->h2d.resizeElapsed;
    }
    return batch;
  }

 public:
  const DeviceArray<SfmObservation>& observations() const {
    return observations_;
  }
  const DeviceArray<SfmSqrtInfo2>& sqrtInfos() const {
    return sqrtInfos_;
  }
  const DeviceArray<SfmRobustModel>& robustModels() const {
    return robustModels_;
  }
  const DeviceArray<int>& pointObservationOffsets() const {
    return pointObservationOffsets_;
  }
  const DeviceArray<int>& longTrackPointSlots() const {
    return longTrackPointSlots_;
  }

  SfmProjectionNoiseMode noiseMode() const { return noiseMode_; }
  bool isWhitened() const {
    return noiseMode_ != SfmProjectionNoiseMode::Unit;
  }
  bool isRobust() const {
    return noiseMode_ == SfmProjectionNoiseMode::Robust;
  }
  size_t numCameras() const { return numCameras_; }
  size_t numPoints() const { return numPoints_; }
  size_t numObservations() const { return observations_.size(); }

 private:
  size_t numCameras_ = 0;
  size_t numPoints_ = 0;
  SfmProjectionNoiseMode noiseMode_ = SfmProjectionNoiseMode::Unit;
  DeviceArray<SfmObservation> observations_;
  DeviceArray<SfmSqrtInfo2> sqrtInfos_;
  DeviceArray<SfmRobustModel> robustModels_;
  DeviceArray<int> pointObservationOffsets_;
  DeviceArray<int> longTrackPointSlots_;
};

}  // namespace gtsam::cuda
