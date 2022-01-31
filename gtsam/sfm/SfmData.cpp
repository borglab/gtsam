/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmData.cpp
 * @date January 2022
 * @author Frank dellaert
 * @brief Data structure for dealing with Structure from Motion data
 */

#include <gtsam/sfm/SfmData.h>

namespace gtsam {

void SfmData::print(const std::string& s) const {
  std::cout << "Number of cameras = " << nrCameras() << std::endl;
  std::cout << "Number of tracks = " << nrTracks() << std::endl;
}

bool SfmData::equals(const SfmData& sfmData, double tol) const {
  // check number of cameras and tracks
  if (nrCameras() != sfmData.nrCameras() || nrTracks() != sfmData.nrTracks()) {
    return false;
  }

  // check each camera
  for (size_t i = 0; i < nrCameras(); ++i) {
    if (!camera(i).equals(sfmData.camera(i), tol)) {
      return false;
    }
  }

  // check each track
  for (size_t j = 0; j < nrTracks(); ++j) {
    if (!track(j).equals(sfmData.track(j), tol)) {
      return false;
    }
  }

  return true;
}

}  // namespace gtsam
