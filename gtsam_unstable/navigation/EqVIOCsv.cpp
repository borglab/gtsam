/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOCsv.cpp
/// @brief CSV utilities for replaying preprocessed EqVIO streams.

#include <gtsam_unstable/navigation/EqVIOCsv.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace gtsam {
namespace eqvio {

namespace {

std::string trim(const std::string& s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> cols;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    cols.push_back(trim(cell));
  }
  return cols;
}

double parseDoubleOr(const std::string& s, double fallback = 0.0) {
  if (s.empty()) return fallback;
  return std::stod(s);
}

size_t parseSizeOr(const std::string& s, size_t fallback = 0) {
  if (s.empty()) return fallback;
  return static_cast<size_t>(std::stoull(s));
}

int parseIntOr(const std::string& s, int fallback = 0) {
  if (s.empty()) return fallback;
  return std::stoi(s);
}

std::unordered_map<std::string, size_t> buildHeaderMap(
    const std::vector<std::string>& header) {
  std::unordered_map<std::string, size_t> index;
  for (size_t i = 0; i < header.size(); ++i) {
    index[header[i]] = i;
  }
  return index;
}

std::string getCell(const std::vector<std::string>& cells,
                    const std::unordered_map<std::string, size_t>& idx,
                    const std::string& key) {
  const auto it = idx.find(key);
  if (it == idx.end()) {
    throw std::invalid_argument("EqVIOCsv: missing required column: " + key);
  }
  return (it->second < cells.size()) ? cells[it->second] : "";
}

}  // namespace

EqVIOCsvLog readEqVIOCsv(const std::string& csvPath) {
  std::ifstream in(csvPath);
  if (!in.is_open()) {
    throw std::invalid_argument("EqVIOCsv: cannot open file: " + csvPath);
  }

  std::string headerLine;
  if (!std::getline(in, headerLine)) {
    throw std::invalid_argument("EqVIOCsv: empty file: " + csvPath);
  }

  const std::vector<std::string> header = splitCsvLine(headerLine);
  const auto index = buildHeaderMap(header);
  const std::vector<std::string> required = {
      "row_type", "t_abs", "seq", "frame_idx", "landmark_id", "gx", "gy",
      "gz",       "ax",    "ay",  "az",        "bgx",         "bgy", "bgz",
      "bax",      "bay",   "baz", "u_norm",    "v_norm",      "key", "value"};
  for (const auto& key : required) {
    if (index.count(key) == 0) {
      throw std::invalid_argument("EqVIOCsv: missing required column: " + key);
    }
  }

  EqVIOCsvLog log;
  std::unordered_map<size_t, size_t> seqToVisionEvent;

  std::string line;
  while (std::getline(in, line)) {
    if (trim(line).empty()) continue;

    const std::vector<std::string> cells = splitCsvLine(line);
    const std::string rowType = getCell(cells, index, "row_type");

    if (rowType == "meta") {
      const std::string key = getCell(cells, index, "key");
      const std::string value = getCell(cells, index, "value");
      if (!key.empty()) {
        log.metadata[key] = value;
      }
      continue;
    }

    if (rowType == "imu") {
      EqVIOCsvEvent event;
      event.type = EqVIOCsvEvent::Type::Imu;
      event.seq = parseSizeOr(getCell(cells, index, "seq"));
      event.tAbs = parseDoubleOr(getCell(cells, index, "t_abs"));
      event.imu.stamp = event.tAbs;
      event.imu.gyr = Vector3(parseDoubleOr(getCell(cells, index, "gx")),
                              parseDoubleOr(getCell(cells, index, "gy")),
                              parseDoubleOr(getCell(cells, index, "gz")));
      event.imu.acc = Vector3(parseDoubleOr(getCell(cells, index, "ax")),
                              parseDoubleOr(getCell(cells, index, "ay")),
                              parseDoubleOr(getCell(cells, index, "az")));
      event.imu.gyrBiasVel =
          Vector3(parseDoubleOr(getCell(cells, index, "bgx")),
                  parseDoubleOr(getCell(cells, index, "bgy")),
                  parseDoubleOr(getCell(cells, index, "bgz")));
      event.imu.accBiasVel =
          Vector3(parseDoubleOr(getCell(cells, index, "bax")),
                  parseDoubleOr(getCell(cells, index, "bay")),
                  parseDoubleOr(getCell(cells, index, "baz")));
      log.events.push_back(event);
      continue;
    }

    if (rowType == "vision_feature") {
      const size_t seq = parseSizeOr(getCell(cells, index, "seq"));
      const double tAbs = parseDoubleOr(getCell(cells, index, "t_abs"));
      const int frameIdx = parseIntOr(getCell(cells, index, "frame_idx"), -1);
      const int id = parseIntOr(getCell(cells, index, "landmark_id"));
      const double u = parseDoubleOr(getCell(cells, index, "u_norm"));
      const double v = parseDoubleOr(getCell(cells, index, "v_norm"));

      auto it = seqToVisionEvent.find(seq);
      if (it == seqToVisionEvent.end()) {
        EqVIOCsvEvent event;
        event.type = EqVIOCsvEvent::Type::VisionFrame;
        event.seq = seq;
        event.tAbs = tAbs;
        event.frameIdx = frameIdx;
        event.vision[id] = Point2(u, v);
        seqToVisionEvent.emplace(seq, log.events.size());
        log.events.push_back(std::move(event));
      } else {
        EqVIOCsvEvent& event = log.events[it->second];
        if (event.frameIdx != frameIdx) {
          throw std::invalid_argument(
              "EqVIOCsv: inconsistent frame_idx for vision seq=" +
              std::to_string(seq));
        }
        if (std::abs(event.tAbs - tAbs) > 1e-9) {
          throw std::invalid_argument(
              "EqVIOCsv: inconsistent t_abs for vision seq=" +
              std::to_string(seq));
        }
        event.vision[id] = Point2(u, v);
      }
      continue;
    }

    throw std::invalid_argument("EqVIOCsv: unknown row_type: " + rowType);
  }

  std::stable_sort(log.events.begin(), log.events.end(),
                   [](const EqVIOCsvEvent& a, const EqVIOCsvEvent& b) {
                     return a.seq < b.seq;
                   });
  return log;
}

double metadataDouble(const EqVIOCsvLog& log, const std::string& key,
                      double fallback) {
  const auto it = log.metadata.find(key);
  if (it == log.metadata.end() || it->second.empty()) return fallback;
  try {
    return std::stod(it->second);
  } catch (...) {
    return fallback;
  }
}

std::optional<Pose3> cameraExtrinsicsFromMetadata(const EqVIOCsvLog& log) {
  const std::vector<std::string> keys = {
      "T_ci_tx", "T_ci_ty", "T_ci_tz", "T_ci_qw",
      "T_ci_qx", "T_ci_qy", "T_ci_qz"};
  for (const auto& key : keys) {
    const auto it = log.metadata.find(key);
    if (it == log.metadata.end() || it->second.empty()) {
      return std::nullopt;
    }
  }

  const double tx = std::stod(log.metadata.at("T_ci_tx"));
  const double ty = std::stod(log.metadata.at("T_ci_ty"));
  const double tz = std::stod(log.metadata.at("T_ci_tz"));
  double qw = std::stod(log.metadata.at("T_ci_qw"));
  double qx = std::stod(log.metadata.at("T_ci_qx"));
  double qy = std::stod(log.metadata.at("T_ci_qy"));
  double qz = std::stod(log.metadata.at("T_ci_qz"));

  const double qnorm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (qnorm <= 1e-12) {
    return std::nullopt;
  }
  qw /= qnorm;
  qx /= qnorm;
  qy /= qnorm;
  qz /= qnorm;

  return Pose3(Rot3::Quaternion(qw, qx, qy, qz), Point3(tx, ty, tz));
}

}  // namespace eqvio
}  // namespace gtsam
