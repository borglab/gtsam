#pragma once

#include <gtsam/sfm/SfmData.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {

class CudaBalCsrStructure {
 public:
  static CudaBalCsrStructure FromSfmData(const SfmData& data) {
    CudaBalCsrStructure structure;
    structure.numCameras_ = data.numberCameras();
    structure.numPoints_ = data.numberTracks();

    constexpr size_t kCameraDim = 9;
    constexpr size_t kPointDim = 3;
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
    std::vector<std::set<int>> rows(static_cast<size_t>(structure.dimension_));

    for (size_t cameraIndex = 0; cameraIndex < structure.numCameras_;
         ++cameraIndex) {
      addDenseBlock(&rows, cameraBase(cameraIndex), cameraBase(cameraIndex), 9,
                    9);
    }

    for (size_t pointIndex = 0; pointIndex < structure.numPoints_;
         ++pointIndex) {
      addDenseBlock(&rows, structure.pointBase(pointIndex),
                    structure.pointBase(pointIndex), 3, 3);
    }

    for (size_t pointIndex = 0; pointIndex < data.numberTracks(); ++pointIndex) {
      const SfmTrack& track = data.track(pointIndex);
      if (track.numberMeasurements() < 2) {
        // Keep original point slots and point diagonals, but only tracks in the
        // projection batch contribute camera-point blocks.
        continue;
      }

      for (const SfmMeasurement& measurement : track.measurements) {
        if (measurement.first >= structure.numCameras_) {
          throw std::invalid_argument(
              "CudaBalCsrStructure camera index out of range");
        }
        addDenseBlock(&rows, cameraBase(measurement.first),
                      structure.pointBase(pointIndex), 9, 3);
      }
    }

    structure.rowPointers_.reserve(static_cast<size_t>(structure.dimension_) +
                                   1);
    structure.rowPointers_.push_back(0);
    for (const std::set<int>& row : rows) {
      structure.colIndices_.insert(structure.colIndices_.end(), row.begin(),
                                   row.end());
      if (structure.colIndices_.size() > kMaxInt) {
        throw std::invalid_argument("CudaBalCsrStructure too many nonzeros");
      }
      structure.rowPointers_.push_back(
          static_cast<int>(structure.colIndices_.size()));
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
  int dimension_ = 0;
  size_t numCameras_ = 0;
  size_t numPoints_ = 0;
  std::vector<int> rowPointers_;
  std::vector<int> colIndices_;

  static int cameraBase(size_t cameraIndex) {
    return static_cast<int>(9 * cameraIndex);
  }

  int pointBase(size_t pointIndex) const {
    return static_cast<int>(9 * numCameras_ + 3 * pointIndex);
  }

  static void addDenseBlock(std::vector<std::set<int>>* rows, int rowBase,
                            int colBase, int rowDim, int colDim) {
    for (int r = 0; r < rowDim; ++r) {
      for (int c = 0; c < colDim; ++c) {
        addEntry(rows, rowBase + r, colBase + c);
      }
    }
  }

  static void addEntry(std::vector<std::set<int>>* rows, int row, int col) {
    if (row > col) {
      std::swap(row, col);
    }
    rows->at(static_cast<size_t>(row)).insert(col);
  }
};

}  // namespace gtsam::cuda
