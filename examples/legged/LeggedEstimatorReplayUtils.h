#pragma once

#include <gtsam/navigation/LeggedEstimator.h>
#include <gtsam/navigation/NavState.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtsam {

namespace fs = std::filesystem;

/// Dataset-level metadata stored next to the staircase example CSV files.
struct DatasetMetadata {
  std::vector<std::string> footNames;
  bool denseContactStream = false;
  int contactStateValue = 1;
  std::string timestampSource = "record";
};

/// One IMU sample from the staircase example.
struct ImuSample {
  size_t index = 0;
  double timestampS = 0.0;
  Vector3 omega = Vector3::Zero();
  Vector3 specificForce = Vector3::Zero();
};

/// One contact event from the staircase example.
struct ContactEvent {
  size_t index = 0;
  double timestampS = 0.0;
  std::vector<ContactMeasurement> activeContacts;
};

/// In-memory representation of the staircase example dataset.
struct Dataset {
  DatasetMetadata metadata;
  std::vector<ImuSample> imuSamples;
  std::vector<ContactEvent> contactEvents;
};

/// Aggregate replay metrics written for one estimator run.
struct ReplayMetrics {
  double wallTimeMs = 0.0;
  double meanContactResidual = 0.0;
  double rmsContactResidual = 0.0;
  double maxContactResidual = 0.0;
  double meanHeightAbsError = 0.0;
  double pathLength = 0.0;
  double loopClosureError = 0.0;
  size_t trajectoryRows = 0;
  size_t contactEvents = 0;
  size_t contactMeasurements = 0;
};

/// Final outputs recorded for one estimator run.
struct ReplayOutputs {
  std::string filterName;
  ReplayMetrics metrics;
  NavState finalState;
};

/// One row in the contact packet usage CSV.
struct ContactPacketUsageRow {
  size_t eventIndex = 0;
  double timestampS = 0.0;
  double elapsedS = 0.0;
  size_t footIndex = 0;
  std::string footName;
  std::string status;
  std::string color;
};

/// Convert a floating-point value to the standard replay CSV formatting.
inline std::string toString(double value, int precision = 12) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

/// Split one CSV line, trimming a trailing CR when present.
inline std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    if (!field.empty() && field.back() == '\r') {
      field.pop_back();
    }
    fields.push_back(field);
  }
  return fields;
}

/// Read all non-empty rows from a CSV file.
inline std::vector<std::vector<std::string>> readCsvRows(const fs::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Unable to open CSV file: " + path.string());
  }

  std::vector<std::vector<std::string>> rows;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    rows.push_back(splitCsvLine(line));
  }
  return rows;
}

/// Parse a floating-point CSV field with a descriptive error.
inline double parseDouble(const std::string& value, const std::string& label) {
  try {
    return std::stod(value);
  } catch (const std::exception&) {
    throw std::runtime_error("Unable to parse " + label + " from value '" +
                             value + "'");
  }
}

/// Parse a size_t CSV field with a descriptive error.
inline size_t parseSize(const std::string& value, const std::string& label) {
  try {
    return static_cast<size_t>(std::stoul(value));
  } catch (const std::exception&) {
    throw std::runtime_error("Unable to parse " + label + " from value '" +
                             value + "'");
  }
}

/// Load dataset metadata from `metadata.csv`.
inline DatasetMetadata loadMetadata(const fs::path& path) {
  const std::vector<std::vector<std::string>> rows = readCsvRows(path);
  if (rows.size() < 2) {
    throw std::runtime_error("Metadata file is empty: " + path.string());
  }

  DatasetMetadata metadata;
  for (size_t rowIndex = 1; rowIndex < rows.size(); ++rowIndex) {
    const std::vector<std::string>& fields = rows[rowIndex];
    if (fields.empty()) {
      continue;
    }
    const std::string& key = fields[0];
    if (key == "foot_names") {
      metadata.footNames.assign(fields.begin() + 1, fields.end());
    } else if (key == "dense_contact_stream") {
      metadata.denseContactStream = parseDouble(fields.at(1), key) != 0.0;
    } else if (key == "contact_state_value") {
      metadata.contactStateValue =
          static_cast<int>(parseDouble(fields.at(1), key));
    } else if (key == "timestamp_source") {
      metadata.timestampSource = fields.at(1);
    }
  }

  if (metadata.footNames.empty()) {
    throw std::runtime_error("metadata.csv did not contain foot_names");
  }
  return metadata;
}

/// Load IMU samples from `imu.csv`.
inline std::vector<ImuSample> loadImuSamples(const fs::path& path) {
  const std::vector<std::vector<std::string>> rows = readCsvRows(path);
  std::vector<ImuSample> samples;
  samples.reserve(rows.size() > 0 ? rows.size() - 1 : 0);
  for (size_t rowIndex = 1; rowIndex < rows.size(); ++rowIndex) {
    const std::vector<std::string>& fields = rows[rowIndex];
    if (fields.size() != 8) {
      throw std::runtime_error("imu.csv row has wrong width");
    }
    ImuSample sample;
    sample.index = parseSize(fields[0], "sample_index");
    sample.timestampS = parseDouble(fields[1], "timestamp_s");
    sample.omega = Vector3(parseDouble(fields[2], "omega_x"),
                           parseDouble(fields[3], "omega_y"),
                           parseDouble(fields[4], "omega_z"));
    sample.specificForce = Vector3(parseDouble(fields[5], "acc_x"),
                                   parseDouble(fields[6], "acc_y"),
                                   parseDouble(fields[7], "acc_z"));
    samples.push_back(sample);
  }
  return samples;
}

/// Load dense contact packets from `contacts.csv`.
inline std::vector<ContactEvent> loadContactEvents(const fs::path& path) {
  const std::vector<std::vector<std::string>> rows = readCsvRows(path);
  std::vector<ContactEvent> events;
  std::optional<size_t> currentIndex;
  for (size_t rowIndex = 1; rowIndex < rows.size(); ++rowIndex) {
    const std::vector<std::string>& fields = rows[rowIndex];
    if (fields.size() != 8) {
      throw std::runtime_error("contacts.csv row has wrong width");
    }
    const size_t eventIndex = parseSize(fields[0], "event_index");
    if (!currentIndex || *currentIndex != eventIndex) {
      events.push_back(
          ContactEvent{eventIndex, parseDouble(fields[1], "timestamp_s"), {}});
      currentIndex = eventIndex;
    }
    ContactMeasurement measurement;
    measurement.foot = parseSize(fields[2], "foot_index");
    measurement.touchdown = parseSize(fields[4], "is_new_contact") != 0;
    measurement.bodyPoint = Vector3(parseDouble(fields[5], "body_x"),
                                    parseDouble(fields[6], "body_y"),
                                    parseDouble(fields[7], "body_z"));
    events.back().activeContacts.push_back(measurement);
  }
  return events;
}

/// Load one staircase example dataset from a CSV directory.
inline Dataset loadDataset(const fs::path& datasetDir) {
  Dataset dataset;
  dataset.metadata = loadMetadata(datasetDir / "metadata.csv");
  dataset.imuSamples = loadImuSamples(datasetDir / "imu.csv");
  dataset.contactEvents = loadContactEvents(datasetDir / "contacts.csv");
  if (dataset.imuSamples.empty()) {
    throw std::runtime_error("Dataset contains no IMU samples");
  }
  return dataset;
}

/// Write the trajectory CSV header.
inline void writeTrajectoryHeader(std::ofstream& output) {
  output << "kind,timestamp_s,x,y,z,qw,qx,qy,qz,roll,pitch,yaw,vx,vy,vz\n";
}

/// Write one trajectory CSV row.
inline void writeTrajectoryRow(std::ofstream& output, const std::string& kind,
                               double timestampS, const NavState& state) {
  const auto quaternion = state.quaternion();
  const Vector3 rpy = state.attitude().rpy();
  output << kind << ',' << toString(timestampS) << ','
         << toString(state.position().x()) << ','
         << toString(state.position().y()) << ','
         << toString(state.position().z()) << ',' << toString(quaternion.w())
         << ',' << toString(quaternion.x()) << ',' << toString(quaternion.y())
         << ',' << toString(quaternion.z()) << ',' << toString(rpy.x()) << ','
         << toString(rpy.y()) << ',' << toString(rpy.z()) << ','
         << toString(state.velocity().x()) << ','
         << toString(state.velocity().y()) << ','
         << toString(state.velocity().z()) << '\n';
}

/// Write the metrics CSV header.
inline void writeMetricsHeader(std::ofstream& output) {
  output << "wall_time_ms,trajectory_rows,contact_events,contact_measurements,"
         << "mean_contact_residual,rms_contact_residual,max_contact_residual,"
         << "mean_height_abs_error,path_length,loop_closure_error,"
         << "final_x,final_y,final_z,"
         << "final_roll,final_pitch,final_yaw\n";
}

/// Write one metrics CSV row.
inline void writeMetricsRow(std::ofstream& output,
                            const ReplayOutputs& outputs) {
  const Vector3 rpy = outputs.finalState.attitude().rpy();
  output << toString(outputs.metrics.wallTimeMs) << ','
         << outputs.metrics.trajectoryRows << ','
         << outputs.metrics.contactEvents << ','
         << outputs.metrics.contactMeasurements << ','
         << toString(outputs.metrics.meanContactResidual) << ','
         << toString(outputs.metrics.rmsContactResidual) << ','
         << toString(outputs.metrics.maxContactResidual) << ','
         << toString(outputs.metrics.meanHeightAbsError) << ','
         << toString(outputs.metrics.pathLength) << ','
         << toString(outputs.metrics.loopClosureError) << ','
         << toString(outputs.finalState.position().x()) << ','
         << toString(outputs.finalState.position().y()) << ','
         << toString(outputs.finalState.position().z()) << ','
         << toString(rpy.x()) << ',' << toString(rpy.y()) << ','
         << toString(rpy.z()) << '\n';
}

/// Write the shared contact packet usage CSV.
inline void writeContactPacketUsage(const fs::path& path,
                                    const std::vector<ContactPacketUsageRow>& rows) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Unable to open contact packet usage output.");
  }

  output << "event_index,timestamp_s,elapsed_s,foot_index,foot_name,status,"
            "color\n";
  for (const ContactPacketUsageRow& row : rows) {
    output << row.eventIndex << ',' << toString(row.timestampS) << ','
           << toString(row.elapsedS) << ',' << row.footIndex << ','
           << row.footName << ',' << row.status << ',' << row.color << '\n';
  }
}

}  // namespace gtsam
