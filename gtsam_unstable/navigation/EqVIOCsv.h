/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOCsv.h
/// @brief CSV utilities for replaying preprocessed EqVIO streams.

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam_unstable/dllexport.h>
#include <gtsam_unstable/navigation/EqVIOCommon.h>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace gtsam {
namespace eqvio {

/// Parsed replay event from a typed EqVIO CSV stream.
struct GTSAM_UNSTABLE_EXPORT EqVIOCsvEvent {
  enum class Type { Imu, VisionFrame };

  Type type = Type::Imu;
  size_t seq = 0;
  double tAbs = 0.0;
  int frameIdx = -1;
  IMUInput imu;
  VisionMeasurement vision;
};

/// Parsed EqVIO CSV log: metadata and merged event stream.
struct GTSAM_UNSTABLE_EXPORT EqVIOCsvLog {
  std::map<std::string, std::string> metadata;
  std::vector<EqVIOCsvEvent> events;
};

/// Parse a typed EqVIO CSV file.
GTSAM_UNSTABLE_EXPORT EqVIOCsvLog readEqVIOCsv(const std::string& csvPath);

/// Parse metadata value as double with fallback.
GTSAM_UNSTABLE_EXPORT double metadataDouble(const EqVIOCsvLog& log,
                                            const std::string& key,
                                            double fallback);

/// Parse camera extrinsics Pose3 from metadata keys, if present.
GTSAM_UNSTABLE_EXPORT std::optional<Pose3> cameraExtrinsicsFromMetadata(
    const EqVIOCsvLog& log);

}  // namespace eqvio
}  // namespace gtsam

