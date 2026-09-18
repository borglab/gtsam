/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TimingUtils.cpp
 * @brief Implementations for private timing utilities.
 */

#include "TimingUtils.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace gtsam::timing {
namespace {

std::runtime_error argumentError(const std::string& message) {
  return std::runtime_error("Invalid command line: " + message);
}

template <class Value, class Parser>
Value parseNumber(const std::string& name, const std::string& text,
                  Parser&& parser) {
  size_t parsedCharacters = 0;
  try {
    const Value value = parser(text, &parsedCharacters);
    if (parsedCharacters != text.size()) {
      throw argumentError("value for " + name + " is not a number: " + text);
    }
    return value;
  } catch (const std::invalid_argument&) {
    throw argumentError("value for " + name + " is not a number: " + text);
  } catch (const std::out_of_range&) {
    throw argumentError("value for " + name + " is out of range: " + text);
  }
}

}  // namespace

Arguments::Arguments(int argc, const char* const argv[]) {
  tokens_.reserve(argc > 1 ? static_cast<size_t>(argc - 1) : 0);
  for (int i = 1; i < argc; ++i) tokens_.emplace_back(argv[i]);
  consumed_.resize(tokens_.size(), false);
}

Arguments::Arguments(std::vector<std::string> tokens)
    : tokens_(std::move(tokens)), consumed_(tokens_.size(), false) {}

bool Arguments::flag(const std::string& name) {
  bool found = false;
  for (size_t i = 0; i < tokens_.size(); ++i) {
    if (!consumed_[i] && tokens_[i] == name) {
      consumed_[i] = true;
      found = true;
    }
  }
  return found;
}

bool Arguments::helpRequested() { return flag("--help"); }

std::optional<std::string> Arguments::optionalString(const std::string& name) {
  std::optional<std::string> result;
  for (size_t i = 0; i < tokens_.size(); ++i) {
    if (consumed_[i] || tokens_[i] != name) continue;
    if (i + 1 >= tokens_.size() || consumed_[i + 1] ||
        tokens_[i + 1].rfind("--", 0) == 0) {
      throw argumentError("missing value after " + name);
    }
    consumed_[i] = true;
    consumed_[i + 1] = true;
    result = tokens_[i + 1];
  }
  return result;
}

std::string Arguments::stringValue(const std::string& name,
                                   const std::string& defaultValue) {
  return optionalString(name).value_or(defaultValue);
}

size_t Arguments::sizeValue(const std::string& name, size_t defaultValue) {
  const auto text = optionalString(name);
  if (!text) return defaultValue;
  if (!text->empty() && text->front() == '-') {
    throw argumentError("value for " + name +
                        " must be non-negative: " + *text);
  }
  return parseNumber<size_t>(
      name, *text, [](const std::string& value, size_t* parsedCharacters) {
        return static_cast<size_t>(std::stoull(value, parsedCharacters));
      });
}

uint64_t Arguments::uint64Value(const std::string& name,
                                uint64_t defaultValue) {
  const auto text = optionalString(name);
  if (!text) return defaultValue;
  if (!text->empty() && text->front() == '-') {
    throw argumentError("value for " + name +
                        " must be non-negative: " + *text);
  }
  return parseNumber<uint64_t>(
      name, *text, [](const std::string& value, size_t* parsedCharacters) {
        return std::stoull(value, parsedCharacters);
      });
}

double Arguments::doubleValue(const std::string& name, double defaultValue) {
  const auto text = optionalString(name);
  if (!text) return defaultValue;
  const double value = parseNumber<double>(
      name, *text, [](const std::string& input, size_t* parsedCharacters) {
        return std::stod(input, parsedCharacters);
      });
  if (!std::isfinite(value)) {
    throw argumentError("value for " + name + " must be finite: " + *text);
  }
  return value;
}

std::vector<std::string> Arguments::positionals() {
  std::vector<std::string> result;
  for (size_t i = 0; i < tokens_.size(); ++i) {
    if (!consumed_[i] && tokens_[i].rfind("-", 0) != 0) {
      result.push_back(tokens_[i]);
      consumed_[i] = true;
    }
  }
  return result;
}

std::vector<size_t> Arguments::sizePositionals() {
  const std::vector<std::string> values = positionals();
  std::vector<size_t> result;
  result.reserve(values.size());
  for (const std::string& value : values) {
    if (!value.empty() && value.front() == '-') {
      throw argumentError("positional value must be non-negative: " + value);
    }
    result.push_back(parseNumber<size_t>(
        "positional argument", value,
        [](const std::string& input, size_t* parsedCharacters) {
          return static_cast<size_t>(std::stoull(input, parsedCharacters));
        }));
  }
  return result;
}

void Arguments::validateAllConsumed() const {
  for (size_t i = 0; i < tokens_.size(); ++i) {
    if (!consumed_[i]) throw argumentError("unknown argument: " + tokens_[i]);
  }
}

TimingSummary summarizeSamples(const std::vector<double>& samples,
                               MedianPolicy medianPolicy) {
  if (samples.empty()) {
    throw std::invalid_argument("Cannot summarize an empty sample set");
  }

  std::vector<double> sorted = samples;
  std::sort(sorted.begin(), sorted.end());

  TimingSummary summary;
  summary.count = sorted.size();
  summary.mean =
      std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size();
  const size_t middle = sorted.size() / 2;
  summary.median = sorted[middle];
  if (medianPolicy == MedianPolicy::kAverageMiddle && sorted.size() % 2 == 0) {
    summary.median = 0.5 * (sorted[middle - 1] + sorted[middle]);
  }

  double squaredDifferenceSum = 0.0;
  for (const double sample : sorted) {
    const double difference = sample - summary.mean;
    squaredDifferenceSum += difference * difference;
  }
  summary.standardDeviation =
      std::sqrt(squaredDifferenceSum / static_cast<double>(sorted.size()));
  summary.minimum = sorted.front();
  summary.maximum = sorted.back();

  // Retain the integer percentile indices used by the existing PCG benchmark.
  const size_t last = sorted.size() - 1;
  summary.p10 = sorted[last / 10];
  summary.p90 = sorted[(9 * last) / 10];
  return summary;
}

std::ofstream openOutputFile(const std::string& path,
                             bool createParentDirectories) {
  const std::filesystem::path outputPath(path);
  const std::filesystem::path parent = outputPath.parent_path();
  if (createParentDirectories && !parent.empty()) {
    std::error_code error;
    std::filesystem::create_directories(parent, error);
    if (error) {
      throw std::runtime_error("Unable to create output directory " +
                               parent.string() + ": " + error.message());
    }
  }

  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Unable to open output file: " + path);
  }
  return output;
}

std::string escapeJson(std::string_view value) {
  std::ostringstream escaped;
  escaped << std::hex << std::setfill('0');
  for (const unsigned char character : value) {
    switch (character) {
      case '\\':
        escaped << "\\\\";
        break;
      case '"':
        escaped << "\\\"";
        break;
      case '\b':
        escaped << "\\b";
        break;
      case '\f':
        escaped << "\\f";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        if (character < 0x20) {
          escaped << "\\u" << std::setw(4) << static_cast<int>(character);
        } else {
          escaped << static_cast<char>(character);
        }
    }
  }
  return escaped.str();
}

std::string serializeBenchmarkActionMetrics(
    const std::vector<BenchmarkMetric>& metrics) {
  std::ostringstream output;
  output << "[\n";
  for (size_t i = 0; i < metrics.size(); ++i) {
    if (i != 0) output << ",\n";
    const BenchmarkMetric& metric = metrics[i];
    output << "  {\n";
    output << "    \"name\": \"" << escapeJson(metric.name) << "\",\n";
    output << "    \"unit\": \"" << escapeJson(metric.unit) << "\",\n";
    output << "    \"value\": " << std::fixed << std::setprecision(9)
           << metric.value << "\n";
    output << "  }";
  }
  output << "\n]\n";
  return output.str();
}

void writeBenchmarkActionMetrics(const std::string& path,
                                 const std::vector<BenchmarkMetric>& metrics) {
  std::ofstream output = openOutputFile(path);
  output << serializeBenchmarkActionMetrics(metrics);
  if (!output) {
    throw std::runtime_error("Unable to write output file: " + path);
  }
}

}  // namespace gtsam::timing
