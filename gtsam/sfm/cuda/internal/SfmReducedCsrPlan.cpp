/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmReducedCsrPlan.cpp
 * @brief   Symbolic plan for the upper-triangular camera Schur complement
 * @author  Ruogu Li
 * @date    Aug 16, 2026
 */

#include <gtsam/sfm/cuda/internal/SfmReducedCsrPlan.h>

#include <gtsam/linear/cuda/internal/BlockOrdering.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>

#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_set>

namespace gtsam::cuda {
namespace {
constexpr int kCameraDimension = 9;

void checkCameraIndex(int camera, int cameraCount, const char* name) {
  if (camera < 0 || camera >= cameraCount) {
    throw std::out_of_range(std::string(name) + " camera index is out of range");
  }
}

void checkLocalIndex(int index, const char* name) {
  if (index < 0 || index >= kCameraDimension) {
    throw std::out_of_range(std::string(name) + " is outside a 9D camera block");
  }
}
}  // namespace

std::uint64_t SfmReducedCsrPlan::pairKey(int first, int second) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(first)) << 32) |
         static_cast<std::uint32_t>(second);
}

std::uint64_t SfmReducedCsrPlan::scalarKey(int row, int column) {
  return pairKey(row, column);
}

SfmReducedCsrPlan::SfmReducedCsrPlan(
    const SfmData& data, const std::vector<Key>& cameraKeys)
    : cameraCount_(static_cast<int>(data.numberCameras())),
      cameraKeys_(cameraKeys) {
  if (data.numberCameras() >
      static_cast<size_t>(std::numeric_limits<int>::max() /
                          kCameraDimension)) {
    throw std::overflow_error("camera Schur dimension exceeds int range");
  }
  if (cameraKeys.size() != data.numberCameras()) {
    throw std::invalid_argument(
        "camera key count does not match SfmData camera count");
  }
  if (std::unordered_set<Key>(cameraKeys.begin(), cameraKeys.end()).size() !=
      cameraKeys.size()) {
    throw std::invalid_argument("camera keys must be unique");
  }

  std::map<Key, size_t> cameraDimensions;
  for (int camera = 0; camera < cameraCount_; ++camera) {
    cameraDimensions.emplace(cameraKeys_[camera], kCameraDimension);
    cameraPairs_.emplace(pairKey(camera, camera), true);
  }
  cameraKeyInfo_ = KeyInfo(
      cameraDimensions, Ordering(cameraKeys_.begin(), cameraKeys_.end()));
  dimension_ = cudaBlockOffsets(cameraKeyInfo_).back();

  for (size_t point = 0; point < data.numberTracks(); ++point) {
    std::vector<int> cameras;
    const SfmTrack& track = data.track(point);
    cameras.reserve(track.numberMeasurements());
    for (size_t measurement = 0; measurement < track.numberMeasurements();
         ++measurement) {
      const size_t camera = track.measurement(measurement).first;
      if (camera >= data.numberCameras()) {
        throw std::invalid_argument(
            "SfmData track references a camera outside the camera list");
      }
      cameras.push_back(static_cast<int>(camera));
    }
    std::sort(cameras.begin(), cameras.end());
    cameras.erase(std::unique(cameras.begin(), cameras.end()), cameras.end());
    for (size_t first = 0; first < cameras.size(); ++first) {
      for (size_t second = first + 1; second < cameras.size(); ++second) {
        cameraPairs_.emplace(pairKey(cameras[first], cameras[second]), true);
      }
    }
  }

  std::vector<std::set<int>> columnsByRow(static_cast<size_t>(dimension_));
  for (const auto& pair : cameraPairs_) {
    const int cameraI = static_cast<int>(pair.first >> 32);
    const int cameraJ = static_cast<int>(pair.first & 0xffffffffU);
    for (int localRow = 0; localRow < kCameraDimension; ++localRow) {
      const int firstColumn = cameraI == cameraJ ? localRow : 0;
      for (int localColumn = firstColumn; localColumn < kCameraDimension;
           ++localColumn) {
        columnsByRow[cameraI * kCameraDimension + localRow].insert(
            cameraJ * kCameraDimension + localColumn);
      }
    }
  }

  rowPointers_.reserve(static_cast<size_t>(dimension_) + 1);
  rowPointers_.push_back(0);
  for (int row = 0; row < dimension_; ++row) {
    for (const int column : columnsByRow[row]) {
      const int offset = static_cast<int>(columnIndices_.size());
      columnIndices_.push_back(column);
      scalarOffsets_.emplace(scalarKey(row, column), offset);
    }
    if (columnIndices_.size() >
        static_cast<size_t>(std::numeric_limits<int>::max())) {
      throw std::overflow_error("camera Schur CSR nonzero count exceeds int range");
    }
    rowPointers_.push_back(static_cast<int>(columnIndices_.size()));
  }
}

bool SfmReducedCsrPlan::hasCameraPair(int cameraI, int cameraJ) const {
  checkCameraIndex(cameraI, cameraCount_, "first");
  checkCameraIndex(cameraJ, cameraCount_, "second");
  if (cameraI > cameraJ) std::swap(cameraI, cameraJ);
  return cameraPairs_.count(pairKey(cameraI, cameraJ)) != 0;
}

int SfmReducedCsrPlan::valueOffset(int cameraI, int cameraJ, int localRow,
                                      int localColumn) const {
  checkCameraIndex(cameraI, cameraCount_, "row");
  checkCameraIndex(cameraJ, cameraCount_, "column");
  checkLocalIndex(localRow, "local row");
  checkLocalIndex(localColumn, "local column");

  int row = cameraI * kCameraDimension + localRow;
  int column = cameraJ * kCameraDimension + localColumn;
  if (row > column) std::swap(row, column);
  const auto found = scalarOffsets_.find(scalarKey(row, column));
  if (found == scalarOffsets_.end()) {
    throw std::invalid_argument(
        "requested camera pair is absent from the Schur sparsity plan");
  }
  return found->second;
}

Ordering SfmReducedCsrPlan::colamdOrdering() const {
  SymbolicFactorGraph graph;
  for (const Key key : cameraKeys_) {
    graph.emplace_shared<SymbolicFactor>(key);
  }
  for (const auto& pair : cameraPairs_) {
    const int cameraI = static_cast<int>(pair.first >> 32);
    const int cameraJ = static_cast<int>(pair.first & 0xffffffffU);
    if (cameraI != cameraJ) {
      graph.emplace_shared<SymbolicFactor>(cameraKeys_[cameraI],
                                           cameraKeys_[cameraJ]);
    }
  }
  return Ordering::Colamd(graph);
}

}  // namespace gtsam::cuda
