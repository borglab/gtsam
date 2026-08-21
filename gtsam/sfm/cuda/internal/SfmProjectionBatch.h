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
#include <gtsam/sfm/cuda/SfmTypes.h>

#include <cuda_runtime_api.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {

/// How much noise machinery a batch carries, which decides which kernels run.
enum class SfmProjectionNoiseMode {
  /// No whitening or reweighting: every measurement has identity information.
  Unit,
  /// Per-measurement whitening by an SfmSqrtInfo2.
  Whitened,
  /// Whitening plus a per-measurement robust loss.
  Robust,
};

/// Cost of building and uploading a batch, filled only when requested.
struct SfmProjectionBatchTransferProfile {
  /// Time flattening SfmData into the host staging vectors.
  double hostBuildElapsed = 0.0;
  /// Time allocating the device arrays, taken from the upload summary.
  double deviceAllocElapsed = 0.0;
  /// Bytes and time of every host-to-device copy the build made.
  DeviceTransferSummary h2d;
};

/**
 * Every image measurement of an SFM problem, flattened onto the device.
 *
 * SfmData stores measurements per track, in scattered per-track vectors that a
 * kernel cannot walk. This flattens them into one array of SfmObservation
 * ordered by track, with a CSR-style offset array giving each track's range, so a
 * kernel can find a landmark's observations from its slot alone. Tracks with
 * fewer than two measurements are dropped, since a landmark seen once is not
 * determined and contributes nothing to the reduced camera system.
 *
 * The noise models come in as parallel per-track vectors and are flattened the
 * same way, so sqrtInfos() and robustModels() are indexed by observation like
 * observations() is. Which of them are present sets the noise mode, and the
 * unit-noise case uploads neither.
 *
 * The batch is built once and reused for every linearization: it holds the
 * measurements, which do not change, and none of the values, which do.
 * Construction validates thoroughly — camera indices in range, counts agreeing,
 * noise entries finite — because a bad index here becomes an out-of-bounds device
 * read with no diagnostic.
 */
class SfmProjectionBatch {
 public:
  /// Above this many measurements, a track's Schur contribution gets a whole
  /// CUDA block instead of one thread. The contribution is quadratic in the
  /// track's length, so the few long tracks would otherwise dominate a kernel
  /// where every other thread has finished. Both kernels test against this, so
  /// each track is handled by exactly one of them.
  static constexpr int kLongTrackMeasurementThreshold = 4;

  /// Flattens and uploads `data` with unit noise on every measurement.
  static SfmProjectionBatch fromSfmData(
      const SfmData& data, cudaStream_t stream = nullptr,
      SfmProjectionBatchTransferProfile* profile = nullptr) {
    return fromSfmDataImpl(data, nullptr, nullptr, stream, profile);
  }

  /// As above, whitening each measurement by its own square root information.
  /// The outer vector is indexed by track and the inner by measurement, matching
  /// SfmData's own layout.
  static SfmProjectionBatch fromSfmData(
      const SfmData& data,
      const std::vector<std::vector<SfmSqrtInfo2>>& sqrtInfoByTrack,
      cudaStream_t stream = nullptr,
      SfmProjectionBatchTransferProfile* profile = nullptr) {
    return fromSfmDataImpl(data, &sqrtInfoByTrack, nullptr, stream, profile);
  }

  /// As above, additionally applying a per-measurement robust loss. Robust
  /// models require square root information, since the loss is applied to the
  /// whitened error.
  static SfmProjectionBatch fromSfmData(
      const SfmData& data,
      const std::vector<std::vector<SfmSqrtInfo2>>& sqrtInfoByTrack,
      const std::vector<std::vector<SfmRobustModel>>& robustModelsByTrack,
      cudaStream_t stream = nullptr,
      SfmProjectionBatchTransferProfile* profile = nullptr) {
    return fromSfmDataImpl(data, &sqrtInfoByTrack, &robustModelsByTrack,
                           stream, profile);
  }

  /// Square root information that leaves a measurement unwhitened.
  static SfmSqrtInfo2 unitSqrtInfo() {
    return SfmSqrtInfo2{1.0, 0.0, 1.0};
  }

  /// Robust model that applies no loss, in the one form isUsableRobustModel()
  /// accepts for SfmRobustModelKind::None.
  static SfmRobustModel noRobustModel() {
    return SfmRobustModel{SfmRobustModelKind::None,
                          SfmRobustReweightScheme::Block, 0.0};
  }

  /// Whether whitening by this would be a no-op, letting a caller skip it.
  static bool isUnitSqrtInfo(const SfmSqrtInfo2& sqrtInfo) {
    return sqrtInfo.r00 == 1.0 && sqrtInfo.r01 == 0.0 &&
           sqrtInfo.r11 == 1.0;
  }

  /// Whether this square root information can be uploaded: finite, with
  /// non-negative diagonal.
  //
  // Zero diagonals are allowed: they encode a zero-information (fully
  // down-weighted) measurement, as produced by GNC weighted graphs.
  static bool isUsableSqrtInfo(const SfmSqrtInfo2& sqrtInfo) {
    return std::isfinite(sqrtInfo.r00) && std::isfinite(sqrtInfo.r01) &&
           std::isfinite(sqrtInfo.r11) && sqrtInfo.r00 >= 0.0 &&
           sqrtInfo.r11 >= 0.0;
  }

  /// Whether this robust model can be uploaded: a loss the kernels implement, a
  /// known reweighting scheme, and a finite positive threshold. None must come
  /// in the canonical noRobustModel() form so a kernel need not special-case it.
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
  /// Shared body of the fromSfmData() overloads; null pointers mean the
  /// corresponding noise data is absent.
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
  /// Every kept measurement, grouped by the track it belongs to.
  const DeviceArray<SfmObservation>& observations() const {
    return observations_;
  }
  /// Whitening per observation, empty when the noise mode is Unit.
  const DeviceArray<SfmSqrtInfo2>& sqrtInfos() const {
    return sqrtInfos_;
  }
  /// Robust loss per observation, empty unless the noise mode is Robust.
  const DeviceArray<SfmRobustModel>& robustModels() const {
    return robustModels_;
  }
  /// CSR-style offsets: landmark `p` owns observations
  /// [pointObservationOffsets[p], pointObservationOffsets[p + 1]). Has one entry
  /// per landmark plus one, and repeats an offset for a dropped track.
  const DeviceArray<int>& pointObservationOffsets() const {
    return pointObservationOffsets_;
  }
  /// Slots of the landmarks with more than kLongTrackMeasurementThreshold
  /// observations, for the block-per-track kernel.
  const DeviceArray<int>& longTrackPointSlots() const {
    return longTrackPointSlots_;
  }

  /// Which noise arrays this batch carries.
  SfmProjectionNoiseMode noiseMode() const { return noiseMode_; }
  /// Whether residuals and Jacobians must be whitened.
  bool isWhitened() const {
    return noiseMode_ != SfmProjectionNoiseMode::Unit;
  }
  /// Whether a robust loss must be applied on top of whitening.
  bool isRobust() const {
    return noiseMode_ == SfmProjectionNoiseMode::Robust;
  }
  /// Cameras in the problem, whether or not each is observed.
  size_t numCameras() const { return numCameras_; }
  /// Landmarks in the problem, including tracks dropped as too short.
  size_t numPoints() const { return numPoints_; }
  /// Measurements actually kept, which is fewer than SfmData holds when tracks
  /// were dropped.
  size_t numObservations() const { return observations_.size(); }

 private:
  /// Camera count, from SfmData rather than from the measurements.
  size_t numCameras_ = 0;
  /// Landmark count, likewise, so slots stay aligned with SfmData's tracks.
  size_t numPoints_ = 0;
  /// Set by which noise arrays construction was given.
  SfmProjectionNoiseMode noiseMode_ = SfmProjectionNoiseMode::Unit;
  /// Flattened measurements, ordered by track.
  DeviceArray<SfmObservation> observations_;
  /// Whitening, parallel to observations_ or empty.
  DeviceArray<SfmSqrtInfo2> sqrtInfos_;
  /// Robust losses, parallel to observations_ or empty.
  DeviceArray<SfmRobustModel> robustModels_;
  /// Per-landmark ranges into observations_.
  DeviceArray<int> pointObservationOffsets_;
  /// Work list for the long-track kernel.
  DeviceArray<int> longTrackPointSlots_;
};

}  // namespace gtsam::cuda
