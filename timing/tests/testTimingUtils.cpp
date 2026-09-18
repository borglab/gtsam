/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testTimingUtils.cpp
 * @brief Tests for the private timing support library.
 */

#include <CppUnitLite/TestHarness.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"

using gtsam::timing::Arguments;
using gtsam::timing::BenchmarkMetric;
using gtsam::timing::MedianPolicy;

namespace {

template <class Callable>
bool throwsRuntimeError(Callable&& callable) {
  try {
    callable();
  } catch (const std::runtime_error&) {
    return true;
  }
  return false;
}

}  // namespace

/* ************************************************************************* */
namespace argument_tests {

// Verifies typed flags, values, repeated values, and positional parsing.
TEST(TimingArguments, ParsesTypedValues) {
  Arguments arguments({"--verbose", "--count", "3", "--ratio", "2.5", "--name",
                       "first", "--name", "last", "input.txt"});
  CHECK(arguments.flag("--verbose"));
  LONGS_EQUAL(3, arguments.sizeValue("--count", 1));
  DOUBLES_EQUAL(2.5, arguments.doubleValue("--ratio", 0.0), 1e-12);
  CHECK(arguments.stringValue("--name", "default") == "last");
  const auto positionals = arguments.positionals();
  LONGS_EQUAL(1, positionals.size());
  CHECK(positionals.front() == "input.txt");
  arguments.validateAllConsumed();
}

// Verifies defaults and explicit help handling.
TEST(TimingArguments, ParsesDefaultsAndHelp) {
  Arguments arguments({"--help"});
  CHECK(arguments.helpRequested());
  CHECK(arguments.stringValue("--name", "default") == "default");
  LONGS_EQUAL(7, arguments.sizeValue("--count", 7));
  arguments.validateAllConsumed();
}

// Verifies positional integers use the same checked numeric conversion.
TEST(TimingArguments, ParsesSizePositionals) {
  Arguments arguments({"32", "64"});
  const auto sizes = arguments.sizePositionals();
  LONGS_EQUAL(2, sizes.size());
  LONGS_EQUAL(32, sizes[0]);
  LONGS_EQUAL(64, sizes[1]);
  arguments.validateAllConsumed();
}

// Verifies missing, malformed, negative, and unknown values are rejected.
TEST(TimingArguments, RejectsInvalidInput) {
  CHECK(throwsRuntimeError([] {
    Arguments arguments({"--count"});
    arguments.sizeValue("--count", 1);
  }));
  CHECK(throwsRuntimeError([] {
    Arguments arguments({"--count", "abc"});
    arguments.sizeValue("--count", 1);
  }));
  CHECK(throwsRuntimeError([] {
    Arguments arguments({"--count", "-1"});
    arguments.sizeValue("--count", 1);
  }));
  CHECK(throwsRuntimeError([] {
    Arguments arguments({"--unknown"});
    arguments.validateAllConsumed();
  }));
}

}  // namespace argument_tests
/* ************************************************************************* */

/* ************************************************************************* */
namespace statistics_tests {

// Verifies both historical even-sample median policies and percentile indices.
TEST(TimingSummary, PreservesMedianPoliciesAndPercentiles) {
  const std::vector<double> samples{10, 2, 7, 4};
  const auto averaged =
      gtsam::timing::summarizeSamples(samples, MedianPolicy::kAverageMiddle);
  const auto upper =
      gtsam::timing::summarizeSamples(samples, MedianPolicy::kUpperMiddle);
  DOUBLES_EQUAL(5.5, averaged.median, 1e-12);
  DOUBLES_EQUAL(7.0, upper.median, 1e-12);
  DOUBLES_EQUAL(2.0, upper.p10, 1e-12);
  DOUBLES_EQUAL(7.0, upper.p90, 1e-12);
  DOUBLES_EQUAL(5.75, upper.mean, 1e-12);
}

// Verifies warmups run untimed and only repetitions produce samples.
TEST(TimingMeasurement, SeparatesWarmupsFromRepetitions) {
  size_t callCount = 0;
  const auto samples =
      gtsam::timing::measureMilliseconds([&callCount] { ++callCount; }, 3, 5);
  LONGS_EQUAL(8, callCount);
  LONGS_EQUAL(5, samples.size());
}

}  // namespace statistics_tests
/* ************************************************************************* */

/* ************************************************************************* */
namespace output_tests {

// Verifies checked output creation makes missing parent directories.
TEST(TimingOutput, CreatesOutputDirectory) {
  const auto base =
      std::filesystem::temp_directory_path() / "gtsam_timing_utils_test";
  const auto outputPath = base / "nested" / "result.txt";
  std::filesystem::remove_all(base);
  {
    auto output = gtsam::timing::openOutputFile(outputPath.string());
    output << "ready";
  }
  CHECK(std::filesystem::exists(outputPath));
  std::filesystem::remove_all(base);
}

// Verifies JSON escaping covers quotes, slashes, whitespace, and controls.
TEST(TimingOutput, EscapesJson) {
  const std::string input = std::string("a\\\"\n\t") + char(1);
  CHECK(gtsam::timing::escapeJson(input) == "a\\\\\\\"\\n\\t\\u0001");
}

// Verifies Benchmark Action JSON formatting remains byte-for-byte stable.
TEST(TimingOutput, SerializesBenchmarkActionExactly) {
  const std::vector<BenchmarkMetric> metrics{{"suite/one", "s", 1.25},
                                             {"quoted\"name", "ms", 2.0}};
  const std::string expected =
      "[\n"
      "  {\n"
      "    \"name\": \"suite/one\",\n"
      "    \"unit\": \"s\",\n"
      "    \"value\": 1.250000000\n"
      "  },\n"
      "  {\n"
      "    \"name\": \"quoted\\\"name\",\n"
      "    \"unit\": \"ms\",\n"
      "    \"value\": 2.000000000\n"
      "  }\n"
      "]\n";
  CHECK(gtsam::timing::serializeBenchmarkActionMetrics(metrics) == expected);
}

}  // namespace output_tests
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
