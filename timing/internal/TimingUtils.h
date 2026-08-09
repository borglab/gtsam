/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TimingUtils.h
 * @brief Private utilities shared by GTSAM timing executables.
 */

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace gtsam::timing {

/** Stateful command-line reader for the small standalone timing programs. */
class Arguments {
 public:
  /** Copy command-line arguments, excluding the executable name. */
  Arguments(int argc, const char* const argv[]);

  /** Construct from argument tokens, primarily for focused unit tests. */
  explicit Arguments(std::vector<std::string> tokens);

  /** Return and consume all occurrences of a Boolean flag. */
  bool flag(const std::string& name);

  /** Return and consume whether `--help` was supplied. */
  bool helpRequested();

  /** Return and consume the last string value, or the supplied default. */
  std::string stringValue(const std::string& name,
                          const std::string& defaultValue);

  /** Return and consume an optional string value. */
  std::optional<std::string> optionalString(const std::string& name);

  /** Return and consume a non-negative integer value. */
  size_t sizeValue(const std::string& name, size_t defaultValue);

  /** Return and consume an unsigned 64-bit integer value. */
  uint64_t uint64Value(const std::string& name, uint64_t defaultValue);

  /** Return and consume a floating-point value. */
  double doubleValue(const std::string& name, double defaultValue);

  /** Return and consume tokens that do not begin with a dash. */
  std::vector<std::string> positionals();

  /** Return and consume positional tokens as non-negative integers. */
  std::vector<size_t> sizePositionals();

  /** Throw when any command-line tokens have not been consumed. */
  void validateAllConsumed() const;

 private:
  std::vector<std::string> tokens_;
  std::vector<bool> consumed_;
};

/** Policy used by existing timing programs for even-sized sample medians. */
enum class MedianPolicy {
  kAverageMiddle,
  kUpperMiddle,
};

/** Common descriptive statistics for a collection of timing samples. */
struct TimingSummary {
  size_t count = 0;
  double mean = 0.0;
  double median = 0.0;
  double standardDeviation = 0.0;
  double minimum = 0.0;
  double maximum = 0.0;
  double p10 = 0.0;
  double p90 = 0.0;
};

/**
 * Summarize samples using population standard deviation and the historical
 * nearest-rank percentile indices used by the timing programs.
 */
TimingSummary summarizeSamples(const std::vector<double>& samples,
                               MedianPolicy medianPolicy);

/** Measure a callable repeatedly in milliseconds after untimed warmups. */
template <class Callable>
std::vector<double> measureMilliseconds(Callable&& callable, size_t warmups,
                                        size_t repetitions) {
  for (size_t i = 0; i < warmups; ++i) {
    std::invoke(callable);
  }

  std::vector<double> samples;
  samples.reserve(repetitions);
  for (size_t i = 0; i < repetitions; ++i) {
    const auto start = std::chrono::steady_clock::now();
    std::invoke(callable);
    const auto end = std::chrono::steady_clock::now();
    samples.push_back(
        std::chrono::duration<double, std::milli>(end - start).count());
  }
  return samples;
}

/** Measure one callable invocation in seconds with a steady clock. */
template <class Callable>
double measureSeconds(Callable&& callable) {
  const auto start = std::chrono::steady_clock::now();
  std::invoke(callable);
  const auto end = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(end - start).count();
}

/** Open an output file and optionally create its missing parent directories. */
std::ofstream openOutputFile(const std::string& path,
                             bool createParentDirectories = true);

/** Escape arbitrary text for use inside a JSON string literal. */
std::string escapeJson(std::string_view value);

/** A single Benchmark Action result entry. */
struct BenchmarkMetric {
  std::string name;
  std::string unit;
  double value = 0.0;
};

/** Serialize metrics using the Benchmark Action JSON schema and formatting. */
std::string serializeBenchmarkActionMetrics(
    const std::vector<BenchmarkMetric>& metrics);

/** Write Benchmark Action metrics to a checked output file. */
void writeBenchmarkActionMetrics(const std::string& path,
                                 const std::vector<BenchmarkMetric>& metrics);

}  // namespace gtsam::timing
