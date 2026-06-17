#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/cuda/CudaSfmTypes.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {

class CudaSfmProjectionBatch {
 public:
  static CudaSfmProjectionBatch FromSfmData(const SfmData& data,
                                            cudaStream_t stream = nullptr) {
    CudaSfmProjectionBatch batch;
    batch.numCameras_ = data.numberCameras();
    batch.numPoints_ = data.numberTracks();

    std::vector<CudaSfmObservation> observations;
    for (size_t pointSlot = 0; pointSlot < data.numberTracks(); ++pointSlot) {
      if (pointSlot > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument(
            "CudaSfmProjectionBatch point index too large");
      }

      const SfmTrack& track = data.track(pointSlot);
      if (track.numberMeasurements() < 2) {
        continue;
      }

      for (const SfmMeasurement& measurement : track.measurements) {
        if (measurement.first >= data.numberCameras() ||
            measurement.first >
                static_cast<size_t>(std::numeric_limits<int>::max())) {
          throw std::invalid_argument(
              "CudaSfmProjectionBatch camera index out of range");
        }

        observations.push_back(
            {static_cast<int>(measurement.first), static_cast<int>(pointSlot),
             measurement.second(0), measurement.second(1)});
      }
    }

    batch.observations_.upload(observations, stream);
    return batch;
  }

  const CudaDeviceArray<CudaSfmObservation>& observations() const {
    return observations_;
  }

  size_t numCameras() const { return numCameras_; }
  size_t numPoints() const { return numPoints_; }
  size_t numObservations() const { return observations_.size(); }

 private:
  size_t numCameras_ = 0;
  size_t numPoints_ = 0;
  CudaDeviceArray<CudaSfmObservation> observations_;
};

}  // namespace gtsam::cuda
