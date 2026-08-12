/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EqVIOReplayHelper.h
 * @brief Streaming CSV replay helpers for EqVIO examples.
 * @author OpenAI Codex
 */

#pragma once

#include <gtsam_unstable/navigation/EqVIOFilter.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace gtsam {
namespace eqvio {
namespace examples {

using MetadataMap = std::map<std::string, std::string>;

/// One replay stream item: either an IMU sample or an aggregated vision frame.
struct ReplayEvent {
  enum class Type { Imu, VisionFrame };

  Type type = Type::Imu;
  size_t seq = 0;
  double tAbs = 0.0;
  int frameIdx = -1;
  IMUInput imu;
  VisionMeasurement vision;
};

/**
 * @brief Prepared IMU propagation data extracted from a rolling IMU buffer.
 *
 * `imuInputs[i]` is held for duration `dts[i]`. `trimCount` indicates how many
 * leading IMU entries can be dropped from the caller-owned buffer after use.
 */
struct BufferedImuPropagation {
  std::vector<IMUInput> imuInputs;
  std::vector<double> dts;
  double propagatedTime = 0.0;
  size_t trimCount = 0;
};

/// Format a consistent exception prefix for EqVIO replay helpers.
inline std::string formatReplayError(const std::string& detail) {
  return "EqVIOReplay: " + detail;
}

/// Trim leading/trailing ASCII whitespace.
inline std::string trim(const std::string& s) {
  size_t begin = 0;
  while (begin < s.size() &&
         std::isspace(static_cast<unsigned char>(s[begin]))) {
    ++begin;
  }

  size_t end = s.size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(s[end - 1]))) {
    --end;
  }
  return s.substr(begin, end - begin);
}

/// Split one CSV line by commas and trim each field.
inline std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> columns;
  std::stringstream stream(line);
  std::string cell;
  while (std::getline(stream, cell, ',')) {
    columns.push_back(trim(cell));
  }
  return columns;
}

/// Parse scalar/string helpers used by CSV rows and metadata.
inline double parseDoubleOr(const std::string& s, double fallback = 0.0) {
  return s.empty() ? fallback : std::stod(s);
}

inline size_t parseSizeOr(const std::string& s, size_t fallback = 0) {
  return s.empty() ? fallback : static_cast<size_t>(std::stoull(s));
}

inline int parseIntOr(const std::string& s, int fallback = 0) {
  return s.empty() ? fallback : std::stoi(s);
}

/// Parse one metadata scalar key with fallback on missing/invalid values.
inline double metadataDouble(const MetadataMap& metadata, const std::string& key,
                             double fallback) {
  const auto it = metadata.find(key);
  if (it == metadata.end() || it->second.empty()) return fallback;
  try {
    return std::stod(it->second);
  } catch (...) {
    return fallback;
  }
}

/// Parse one metadata scalar key and require finite result, otherwise fallback.
inline double metadataFiniteDouble(const MetadataMap& metadata,
                                   const std::string& key, double fallback) {
  const double value = metadataDouble(metadata, key, fallback);
  return std::isfinite(value) ? value : fallback;
}

/**
 * @brief Parse camera extrinsics from metadata keys `T_ci_*` if available.
 * @return `std::nullopt` when any extrinsic field is missing or quaternion is
 * invalid.
 */
inline std::optional<Pose3> cameraExtrinsicsFromMetadata(
    const MetadataMap& metadata) {
  const std::array<const char*, 7> keys = {"T_ci_tx", "T_ci_ty", "T_ci_tz",
                                           "T_ci_qw", "T_ci_qx", "T_ci_qy",
                                           "T_ci_qz"};
  for (const char* key : keys) {
    const auto it = metadata.find(key);
    if (it == metadata.end() || it->second.empty()) return std::nullopt;
  }

  const double tx = std::stod(metadata.at("T_ci_tx"));
  const double ty = std::stod(metadata.at("T_ci_ty"));
  const double tz = std::stod(metadata.at("T_ci_tz"));
  double qw = std::stod(metadata.at("T_ci_qw"));
  double qx = std::stod(metadata.at("T_ci_qx"));
  double qy = std::stod(metadata.at("T_ci_qy"));
  double qz = std::stod(metadata.at("T_ci_qz"));

  const double qnorm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (qnorm <= 1e-12) return std::nullopt;
  qw /= qnorm;
  qx /= qnorm;
  qy /= qnorm;
  qz /= qnorm;

  return Pose3(Rot3::Quaternion(qw, qx, qy, qz), Point3(tx, ty, tz));
}

/// Read only the metadata rows from the replay CSV.
inline MetadataMap readReplayMetadata(const std::string& csvPath) {
  std::ifstream input(csvPath);
  if (!input.is_open()) {
    throw std::invalid_argument(formatReplayError("cannot open file: " + csvPath));
  }

  std::string headerLine;
  if (!std::getline(input, headerLine)) {
    throw std::invalid_argument(formatReplayError("empty file: " + csvPath));
  }

  const std::vector<std::string> header = splitCsvLine(headerLine);
  std::unordered_map<std::string, size_t> index;
  for (size_t i = 0; i < header.size(); ++i) {
    index[header[i]] = i;
  }

  const auto getCell = [&](const std::vector<std::string>& cells,
                           const std::string& key) -> std::string {
    const auto it = index.find(key);
    if (it == index.end()) {
      throw std::invalid_argument(formatReplayError("missing required column: " +
                                                   key));
    }
    return it->second < cells.size() ? cells[it->second] : "";
  };

  MetadataMap metadata;
  std::string line;
  while (std::getline(input, line)) {
    if (trim(line).empty()) continue;
    const std::vector<std::string> cells = splitCsvLine(line);
    if (getCell(cells, "row_type") != "meta") continue;

    const std::string key = getCell(cells, "key");
    if (!key.empty()) {
      metadata[key] = getCell(cells, "value");
    }
  }
  return metadata;
}

/// Stream replay events from the CSV without materializing the full log.
class ReplayReader {
 public:
  explicit ReplayReader(const std::string& csvPath) : input_(csvPath) {
    if (!input_.is_open()) {
      throw std::invalid_argument(formatReplayError("cannot open file: " + csvPath));
    }

    std::string headerLine;
    if (!std::getline(input_, headerLine)) {
      throw std::invalid_argument(formatReplayError("empty file: " + csvPath));
    }

    const std::vector<std::string> header = splitCsvLine(headerLine);
    for (size_t i = 0; i < header.size(); ++i) {
      index_[header[i]] = i;
    }

    const std::array<const char*, 21> required = {
        "row_type", "t_abs", "seq", "frame_idx", "landmark_id", "gx",  "gy",
        "gz",       "ax",    "ay",  "az",        "bgx",         "bgy", "bgz",
        "bax",      "bay",   "baz", "u_norm",    "v_norm",      "key", "value"};
    for (const char* key : required) {
      if (index_.count(key) == 0) {
        throw std::invalid_argument(formatReplayError(
            "missing required column: " + std::string(key)));
      }
    }
  }

  bool next(ReplayEvent& event) {
    std::vector<std::string> cells;
    if (!readNextEventRow(cells)) return false;

    const std::string rowType = getCell(cells, "row_type");
    if (rowType == "imu") {
      event = parseImuEvent(cells);
      return true;
    }
    if (rowType != "vision_feature") {
      throw std::invalid_argument(formatReplayError("unknown row_type: " + rowType));
    }

    event = parseVisionFrame(cells);
    return true;
  }

 private:
  std::ifstream input_;
  std::unordered_map<std::string, size_t> index_;
  std::optional<std::vector<std::string>> bufferedCells_;

  std::string getCell(const std::vector<std::string>& cells,
                      const std::string& key) const {
    const auto it = index_.find(key);
    if (it == index_.end()) {
      throw std::invalid_argument(formatReplayError("missing required column: " +
                                                   key));
    }
    return it->second < cells.size() ? cells[it->second] : "";
  }

  bool readNextEventRow(std::vector<std::string>& cells) {
    if (bufferedCells_) {
      cells = std::move(*bufferedCells_);
      bufferedCells_.reset();
      return true;
    }

    std::string line;
    while (std::getline(input_, line)) {
      if (trim(line).empty()) continue;
      cells = splitCsvLine(line);
      if (getCell(cells, "row_type") == "meta") continue;
      return true;
    }
    return false;
  }

  ReplayEvent parseImuEvent(const std::vector<std::string>& cells) const {
    ReplayEvent event;
    event.type = ReplayEvent::Type::Imu;
    event.seq = parseSizeOr(getCell(cells, "seq"));
    event.tAbs = parseDoubleOr(getCell(cells, "t_abs"));
    event.imu.stamp = event.tAbs;
    event.imu.gyr = Vector3(parseDoubleOr(getCell(cells, "gx")),
                            parseDoubleOr(getCell(cells, "gy")),
                            parseDoubleOr(getCell(cells, "gz")));
    event.imu.acc = Vector3(parseDoubleOr(getCell(cells, "ax")),
                            parseDoubleOr(getCell(cells, "ay")),
                            parseDoubleOr(getCell(cells, "az")));
    event.imu.gyrBiasVel = Vector3(parseDoubleOr(getCell(cells, "bgx")),
                                   parseDoubleOr(getCell(cells, "bgy")),
                                   parseDoubleOr(getCell(cells, "bgz")));
    event.imu.accBiasVel = Vector3(parseDoubleOr(getCell(cells, "bax")),
                                   parseDoubleOr(getCell(cells, "bay")),
                                   parseDoubleOr(getCell(cells, "baz")));
    return event;
  }

  ReplayEvent parseVisionFrame(const std::vector<std::string>& firstCells) {
    ReplayEvent event;
    event.type = ReplayEvent::Type::VisionFrame;
    event.seq = parseSizeOr(getCell(firstCells, "seq"));
    event.tAbs = parseDoubleOr(getCell(firstCells, "t_abs"));
    event.frameIdx = parseIntOr(getCell(firstCells, "frame_idx"), -1);

    appendVisionFeature(firstCells, event);

    std::vector<std::string> cells;
    while (readNextEventRow(cells)) {
      if (getCell(cells, "row_type") != "vision_feature" ||
          parseSizeOr(getCell(cells, "seq")) != event.seq) {
        bufferedCells_ = std::move(cells);
        break;
      }
      appendVisionFeature(cells, event);
    }

    return event;
  }

  void appendVisionFeature(const std::vector<std::string>& cells,
                           ReplayEvent& event) const {
    const double tAbs = parseDoubleOr(getCell(cells, "t_abs"));
    const int frameIdx = parseIntOr(getCell(cells, "frame_idx"), -1);
    if (std::abs(event.tAbs - tAbs) > 1e-9) {
      throw std::invalid_argument(formatReplayError(
          "inconsistent t_abs for vision seq=" + std::to_string(event.seq)));
    }
    if (event.frameIdx != frameIdx) {
      throw std::invalid_argument(formatReplayError(
          "inconsistent frame_idx for vision seq=" + std::to_string(event.seq)));
    }

    const Key id =
        static_cast<Key>(parseSizeOr(getCell(cells, "landmark_id")));
    event.vision[id] = Point2(parseDoubleOr(getCell(cells, "u_norm")),
                              parseDoubleOr(getCell(cells, "v_norm")));
  }
};

/**
 * @brief Convert buffered absolute-timestamp IMU samples into hold segments.
 *
 * The returned segments cover `[currentTime, targetTime]` by clipping each hold
 * interval and preserving the existing replay semantics used by the example.
 */
inline BufferedImuPropagation makeBufferedImuPropagation(
    const std::vector<IMUInput>& imuBuffer, double currentTime,
    double targetTime) {
  BufferedImuPropagation out;
  if (imuBuffer.empty() || targetTime <= currentTime) {
    return out;
  }

  out.imuInputs.reserve(imuBuffer.size());
  out.dts.reserve(imuBuffer.size());
  for (size_t i = 0; i < imuBuffer.size(); ++i) {
    const double t0 = std::max(imuBuffer[i].stamp, currentTime);
    const double t1 = i + 1 < imuBuffer.size()
                          ? std::min(imuBuffer[i + 1].stamp, targetTime)
                          : targetTime;
    const double dt = std::max(t1 - t0, 0.0);
    if (dt <= 0.0) continue;
    out.imuInputs.push_back(imuBuffer[i]);
    out.dts.push_back(dt);
    out.propagatedTime += dt;
  }

  auto it = std::find_if(
      imuBuffer.begin(), imuBuffer.end(),
      [targetTime](const IMUInput& imu) { return imu.stamp >= targetTime; });
  if (it != imuBuffer.begin()) {
    --it;
    out.trimCount = static_cast<size_t>(std::distance(imuBuffer.begin(), it));
  }
  return out;
}

}  // namespace examples
}  // namespace eqvio
}  // namespace gtsam
