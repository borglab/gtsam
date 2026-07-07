#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/cuda/CudaSfmTypes.h>

#include <cuda_runtime_api.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {

enum class CudaSfmProjectionNoiseMode {
  Unit,
  Whitened,
  Robust,
};

struct CudaSfmProjectionBatchTransferProfile {
  double hostBuildElapsed = 0.0;
  double deviceAllocElapsed = 0.0;
  CudaDeviceTransferSummary h2d;
};

class CudaSfmProjectionBatch {
 public:
  static constexpr int kLongTrackMeasurementThreshold = 4;

  static CudaSfmProjectionBatch FromSfmData(
      const SfmData& data, cudaStream_t stream = nullptr,
      CudaSfmProjectionBatchTransferProfile* profile = nullptr) {
    return FromSfmDataImpl(data, nullptr, nullptr, stream, profile);
  }

  static CudaSfmProjectionBatch FromSfmData(
      const SfmData& data,
      const std::vector<std::vector<CudaSfmSqrtInfo2>>& sqrtInfoByTrack,
      cudaStream_t stream = nullptr,
      CudaSfmProjectionBatchTransferProfile* profile = nullptr) {
    return FromSfmDataImpl(data, &sqrtInfoByTrack, nullptr, stream, profile);
  }

  static CudaSfmProjectionBatch FromSfmData(
      const SfmData& data,
      const std::vector<std::vector<CudaSfmSqrtInfo2>>& sqrtInfoByTrack,
      const std::vector<std::vector<CudaSfmRobustModel>>& robustModelsByTrack,
      cudaStream_t stream = nullptr,
      CudaSfmProjectionBatchTransferProfile* profile = nullptr) {
    return FromSfmDataImpl(data, &sqrtInfoByTrack, &robustModelsByTrack,
                           stream, profile);
  }

  static CudaSfmSqrtInfo2 UnitSqrtInfo() {
    return CudaSfmSqrtInfo2{1.0, 0.0, 1.0};
  }

  static CudaSfmRobustModel NoRobustModel() {
    return CudaSfmRobustModel{CudaSfmRobustModelKind::None,
                              CudaSfmRobustReweightScheme::Block, 0.0};
  }

  static bool IsUnitSqrtInfo(const CudaSfmSqrtInfo2& sqrtInfo) {
    return sqrtInfo.r00 == 1.0 && sqrtInfo.r01 == 0.0 &&
           sqrtInfo.r11 == 1.0;
  }

  // Zero diagonals are allowed: they encode a zero-information (fully
  // down-weighted) measurement, as produced by GNC weighted graphs.
  static bool IsUsableSqrtInfo(const CudaSfmSqrtInfo2& sqrtInfo) {
    return std::isfinite(sqrtInfo.r00) && std::isfinite(sqrtInfo.r01) &&
           std::isfinite(sqrtInfo.r11) && sqrtInfo.r00 >= 0.0 &&
           sqrtInfo.r11 >= 0.0;
  }

  static bool IsUsableRobustModel(const CudaSfmRobustModel& robustModel) {
    if (robustModel.kind == CudaSfmRobustModelKind::None) {
      return robustModel.reweightScheme == CudaSfmRobustReweightScheme::Block;
    }
    const bool knownModel =
        robustModel.kind == CudaSfmRobustModelKind::Huber ||
        robustModel.kind == CudaSfmRobustModelKind::Tukey;
    const bool knownReweight =
        robustModel.reweightScheme == CudaSfmRobustReweightScheme::Scalar ||
        robustModel.reweightScheme == CudaSfmRobustReweightScheme::Block;
    return knownModel && knownReweight && std::isfinite(robustModel.parameter) &&
           robustModel.parameter > 0.0;
  }

 private:
  static CudaSfmProjectionBatch FromSfmDataImpl(
      const SfmData& data,
      const std::vector<std::vector<CudaSfmSqrtInfo2>>* sqrtInfoByTrack,
      const std::vector<std::vector<CudaSfmRobustModel>>* robustModelsByTrack,
      cudaStream_t stream,
      CudaSfmProjectionBatchTransferProfile* profile) {
    if (profile) {
      *profile = CudaSfmProjectionBatchTransferProfile{};
    }
    const auto hostBuildStart = std::chrono::steady_clock::now();
    CudaSfmProjectionBatch batch;
    batch.numCameras_ = data.numberCameras();
    batch.numPoints_ = data.numberTracks();
    batch.noiseMode_ =
        robustModelsByTrack ? CudaSfmProjectionNoiseMode::Robust
        : sqrtInfoByTrack  ? CudaSfmProjectionNoiseMode::Whitened
                           : CudaSfmProjectionNoiseMode::Unit;

    if (sqrtInfoByTrack && sqrtInfoByTrack->size() != data.numberTracks()) {
      throw std::invalid_argument(
          "CudaSfmProjectionBatch sqrt-info track count mismatch");
    }
    if (robustModelsByTrack) {
      if (!sqrtInfoByTrack) {
        throw std::invalid_argument(
            "CudaSfmProjectionBatch robust models require sqrt-info");
      }
      if (robustModelsByTrack->size() != data.numberTracks()) {
        throw std::invalid_argument(
            "CudaSfmProjectionBatch robust model track count mismatch");
      }
    }

    std::vector<CudaSfmObservation> observations;
    std::vector<CudaSfmSqrtInfo2> sqrtInfos;
    std::vector<CudaSfmRobustModel> robustModels;
    std::vector<int> pointObservationOffsets;
    std::vector<int> longTrackPointSlots;
    pointObservationOffsets.reserve(data.numberTracks() + 1);
    pointObservationOffsets.push_back(0);
    for (size_t pointSlot = 0; pointSlot < data.numberTracks(); ++pointSlot) {
      if (pointSlot > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument(
            "CudaSfmProjectionBatch point index too large");
      }

      const SfmTrack& track = data.track(pointSlot);
      if (sqrtInfoByTrack &&
          (*sqrtInfoByTrack)[pointSlot].size() !=
              track.numberMeasurements()) {
        throw std::invalid_argument(
            "CudaSfmProjectionBatch sqrt-info measurement count mismatch");
      }
      if (robustModelsByTrack &&
          (*robustModelsByTrack)[pointSlot].size() !=
              track.numberMeasurements()) {
        throw std::invalid_argument(
            "CudaSfmProjectionBatch robust model measurement count mismatch");
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
              "CudaSfmProjectionBatch camera index out of range");
        }

        observations.push_back(
            {static_cast<int>(measurement.first), static_cast<int>(pointSlot),
             measurement.second(0), measurement.second(1)});
        if (sqrtInfoByTrack) {
          const CudaSfmSqrtInfo2 sqrtInfo =
              (*sqrtInfoByTrack)[pointSlot][measurementSlot];
          if (!IsUsableSqrtInfo(sqrtInfo)) {
            throw std::invalid_argument(
                "CudaSfmProjectionBatch sqrt-info contains invalid values");
          }
          sqrtInfos.push_back(sqrtInfo);
        }
        if (robustModelsByTrack) {
          const CudaSfmRobustModel robustModel =
              (*robustModelsByTrack)[pointSlot][measurementSlot];
          if (!IsUsableRobustModel(robustModel)) {
            throw std::invalid_argument(
                "CudaSfmProjectionBatch robust model contains invalid values");
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
            "CudaSfmProjectionBatch sqrt-info/observation count mismatch");
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
            "CudaSfmProjectionBatch robust model/observation count mismatch");
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
  const CudaDeviceArray<CudaSfmObservation>& observations() const {
    return observations_;
  }
  const CudaDeviceArray<CudaSfmSqrtInfo2>& sqrtInfos() const {
    return sqrtInfos_;
  }
  const CudaDeviceArray<CudaSfmRobustModel>& robustModels() const {
    return robustModels_;
  }
  const CudaDeviceArray<int>& pointObservationOffsets() const {
    return pointObservationOffsets_;
  }
  const CudaDeviceArray<int>& longTrackPointSlots() const {
    return longTrackPointSlots_;
  }

  CudaSfmProjectionNoiseMode noiseMode() const { return noiseMode_; }
  bool isWhitened() const {
    return noiseMode_ != CudaSfmProjectionNoiseMode::Unit;
  }
  bool isRobust() const {
    return noiseMode_ == CudaSfmProjectionNoiseMode::Robust;
  }
  size_t numCameras() const { return numCameras_; }
  size_t numPoints() const { return numPoints_; }
  size_t numObservations() const { return observations_.size(); }

 private:
  size_t numCameras_ = 0;
  size_t numPoints_ = 0;
  CudaSfmProjectionNoiseMode noiseMode_ = CudaSfmProjectionNoiseMode::Unit;
  CudaDeviceArray<CudaSfmObservation> observations_;
  CudaDeviceArray<CudaSfmSqrtInfo2> sqrtInfos_;
  CudaDeviceArray<CudaSfmRobustModel> robustModels_;
  CudaDeviceArray<int> pointObservationOffsets_;
  CudaDeviceArray<int> longTrackPointSlots_;
};

}  // namespace gtsam::cuda
