#pragma once

#include <gtsam/sfm/SfmData.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {

class CudaBalCsrStructure {
 public:
  static CudaBalCsrStructure FromSfmData(const SfmData& data) {
    CudaBalCsrStructure structure;
    structure.numCameras_ = data.numberCameras();
    structure.numPoints_ = data.numberTracks();

    constexpr size_t kMaxInt =
        static_cast<size_t>(std::numeric_limits<int>::max());
    if (structure.numCameras_ > kMaxInt / kCameraDim ||
        structure.numPoints_ > kMaxInt / kPointDim) {
      throw std::invalid_argument("CudaBalCsrStructure dimension too large");
    }
    const size_t cameraDimension = kCameraDim * structure.numCameras_;
    const size_t pointDimension = kPointDim * structure.numPoints_;
    if (cameraDimension > kMaxInt - pointDimension) {
      throw std::invalid_argument("CudaBalCsrStructure dimension too large");
    }

    const size_t dimension = cameraDimension + pointDimension;
    structure.dimension_ = static_cast<int>(dimension);

    std::vector<std::vector<int>> cameraPoints(structure.numCameras_);
    structure.collectCameraPointAdjacency(data, &cameraPoints);

    structure.rowPointers_.resize(static_cast<size_t>(structure.dimension_) +
                                  1);
    size_t nnz = 0;
    structure.rowPointers_[0] = 0;
    for (size_t cameraIndex = 0; cameraIndex < structure.numCameras_;
         ++cameraIndex) {
      const size_t cameraPointCount = cameraPoints[cameraIndex].size();
      for (int row = 0; row < kCameraDim; ++row) {
        nnz += static_cast<size_t>(kCameraDim - row) +
               static_cast<size_t>(kPointDim) * cameraPointCount;
        if (nnz > kMaxInt) {
          throw std::invalid_argument("CudaBalCsrStructure too many nonzeros");
        }
        structure.rowPointers_[static_cast<size_t>(cameraBase(cameraIndex)) +
                               row + 1] = static_cast<int>(nnz);
      }
    }
    for (size_t pointIndex = 0; pointIndex < structure.numPoints_;
         ++pointIndex) {
      for (int row = 0; row < kPointDim; ++row) {
        nnz += static_cast<size_t>(kPointDim - row);
        if (nnz > kMaxInt) {
          throw std::invalid_argument("CudaBalCsrStructure too many nonzeros");
        }
        structure.rowPointers_[static_cast<size_t>(
                                   structure.pointBase(pointIndex)) +
                               row + 1] = static_cast<int>(nnz);
      }
    }

    structure.colIndices_.resize(nnz);
    size_t offset = 0;
    for (size_t cameraIndex = 0; cameraIndex < structure.numCameras_;
         ++cameraIndex) {
      const int base = cameraBase(cameraIndex);
      const std::vector<int>& points = cameraPoints[cameraIndex];
      for (int row = 0; row < kCameraDim; ++row) {
        for (int col = row; col < kCameraDim; ++col) {
          structure.colIndices_[offset++] = base + col;
        }
        for (const int pointIndex : points) {
          const int pointBase =
              structure.pointBase(static_cast<size_t>(pointIndex));
          for (int col = 0; col < kPointDim; ++col) {
            structure.colIndices_[offset++] = pointBase + col;
          }
        }
      }
    }
    for (size_t pointIndex = 0; pointIndex < structure.numPoints_;
         ++pointIndex) {
      const int base = structure.pointBase(pointIndex);
      for (int row = 0; row < kPointDim; ++row) {
        for (int col = row; col < kPointDim; ++col) {
          structure.colIndices_[offset++] = base + col;
        }
      }
    }
    if (offset != structure.colIndices_.size()) {
      throw std::logic_error("CudaBalCsrStructure fill mismatch");
    }

    return structure;
  }

  int dimension() const { return dimension_; }
  size_t numCameras() const { return numCameras_; }
  size_t numPoints() const { return numPoints_; }
  const std::vector<int>& rowPointers() const { return rowPointers_; }
  const std::vector<int>& colIndices() const { return colIndices_; }

  bool hasEntry(int row, int col) const {
    if (row < 0 || col < 0 || row >= dimension_ || col >= dimension_) {
      return false;
    }
    if (row > col) {
      std::swap(row, col);
    }

    const auto begin = colIndices_.begin() + rowPointers_[row];
    const auto end = colIndices_.begin() + rowPointers_[row + 1];
    return std::binary_search(begin, end, col);
  }

 private:
  static constexpr int kCameraDim = 9;
  static constexpr int kPointDim = 3;

  int dimension_ = 0;
  size_t numCameras_ = 0;
  size_t numPoints_ = 0;
  std::vector<int> rowPointers_;
  std::vector<int> colIndices_;

  static int cameraBase(size_t cameraIndex) {
    return static_cast<int>(9 * cameraIndex);
  }

  int pointBase(size_t pointIndex) const {
    return static_cast<int>(kCameraDim * numCameras_ + kPointDim * pointIndex);
  }

  void collectCameraPointAdjacency(
      const SfmData& data, std::vector<std::vector<int>>* cameraPoints) const {
    for (size_t pointIndex = 0; pointIndex < data.numberTracks(); ++pointIndex) {
      const SfmTrack& track = data.track(pointIndex);
      if (track.numberMeasurements() < 2) {
        // Keep original point slots and point diagonals, but only tracks in the
        // projection batch contribute camera-point blocks.
        continue;
      }

      for (const SfmMeasurement& measurement : track.measurements) {
        if (measurement.first >= numCameras_) {
          throw std::invalid_argument(
              "CudaBalCsrStructure camera index out of range");
        }
        cameraPoints->at(measurement.first)
            .push_back(static_cast<int>(pointIndex));
      }
    }

    for (std::vector<int>& points : *cameraPoints) {
      std::sort(points.begin(), points.end());
      points.erase(std::unique(points.begin(), points.end()), points.end());
    }
  }
};

}  // namespace gtsam::cuda
