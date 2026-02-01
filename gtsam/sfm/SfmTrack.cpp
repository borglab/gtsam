/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmTrack.cpp
 * @date January 2022
 * @author Frank Dellaert
 * @brief A simple data structure for a track in Structure from Motion
 */

#include <gtsam/sfm/SfmTrack.h>

#include <iostream>

namespace gtsam {

void SfmTrack::print(const std::string& s) const {
  std::cout << "Track with " << measurements.size();
  std::cout << " measurements of point " << p << std::endl;
}

bool SfmTrack::equals(const SfmTrack& sfmTrack, double tol) const {
  // check the 3D point
  if (!p.isApprox(sfmTrack.p)) {
    return false;
  }

  // check the RGB values
  if (r != sfmTrack.r || g != sfmTrack.g || b != sfmTrack.b) {
    return false;
  }

  // compare size of vectors for measurements and siftIndices
  if (numberMeasurements() != sfmTrack.numberMeasurements() ||
      siftIndices.size() != sfmTrack.siftIndices.size()) {
    return false;
  }

  // compare measurements (order sensitive)
  for (size_t idx = 0; idx < numberMeasurements(); ++idx) {
    SfmMeasurement measurement = measurements[idx];
    SfmMeasurement otherMeasurement = sfmTrack.measurements[idx];

    if (measurement.first != otherMeasurement.first ||
        !measurement.second.isApprox(otherMeasurement.second)) {
      return false;
    }
  }

  // compare sift indices (order sensitive)
  for (size_t idx = 0; idx < siftIndices.size(); ++idx) {
    SiftIndex index = siftIndices[idx];
    SiftIndex otherIndex = sfmTrack.siftIndices[idx];

    if (index.first != otherIndex.first || index.second != otherIndex.second) {
      return false;
    }
  }

  return true;
}

}  // namespace gtsam
