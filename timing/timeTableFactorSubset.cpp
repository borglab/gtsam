/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file timeTableFactorSubset.cpp
 * @brief Benchmark products and quotients with a factor on a subset of keys.
 */

#include <gtsam/discrete/TableFactor.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"

namespace {

using gtsam::DiscreteKeys;
using gtsam::DiscreteValues;
using gtsam::TableFactor;

volatile double timingChecksum = 0.0;

// The first key is the most significant bit in these binary-key tables.
size_t projectIndex(size_t index, size_t numberKeys,
                    const DiscreteKeys& subset) {
  size_t projected = 0;
  for (const auto& [key, cardinality] : subset)
    projected = 2 * projected + ((index >> (numberKeys - 1 - key)) & 1);
  return projected;
}

// Check every assignment against arithmetic on the original dense vectors.
// This happens outside timing and does not use TableFactor's index projection.
void validate(const TableFactor& result, const std::vector<double>& values,
              const std::vector<double>& marginal, size_t numberKeys,
              const DiscreteKeys& subset, bool divide) {
  DiscreteValues assignment;
  for (size_t key = 0; key < numberKeys; ++key) assignment[key] = 0;
  for (size_t index = 0; index < values.size(); ++index) {
    for (size_t key = 0; key < numberKeys; ++key)
      assignment[key] = (index >> (numberKeys - 1 - key)) & 1;
    const double denominator = marginal[projectIndex(index, numberKeys, subset)];
    const double expected = divide
        ? (denominator == 0 ? 0 : values[index] / denominator)
        : values[index] * denominator;
    const double actual = result(assignment);
    if (!std::isfinite(actual) ||
        std::abs(actual - expected) > 1e-12 * std::max(1.0, expected))
      throw std::runtime_error("Subset operation disagrees with dense arithmetic");
  }
}

}  // namespace

int main(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  if (arguments.helpRequested()) {
    std::cout << "Usage: timeTableFactorSubset [--warmup N] [--repeats N] "
                 "[--output FILE]\n"
                 "Runs dense and sparse tables with 6, 10, and 14 binary keys.\n";
    return 0;
  }
  const size_t warmups = arguments.sizeValue("--warmup", 3);
  const size_t repetitions = arguments.sizeValue("--repeats", 31);
  const std::string outputPath = arguments.stringValue("--output", "");
  arguments.validateAllConsumed();
  if (repetitions == 0) throw std::invalid_argument("require repeats >= 1");

  std::vector<gtsam::timing::BenchmarkMetric> metrics;
  std::cout << "keys,sparse,scope,operation,median_us\n"
            << std::fixed << std::setprecision(3);
  for (size_t numberKeys : {6, 10, 14}) {
    DiscreteKeys keys;
    for (size_t key = 0; key < numberKeys; ++key) keys.emplace_back(key, 2);
    for (bool sparse : {false, true}) {
      std::vector<double> values(size_t(1) << numberKeys);
      for (size_t index = 0; index < values.size(); ++index)
        values[index] = sparse && index % 5 ? 0 : 1 + index % 7;
      const TableFactor joint(keys, values);
      for (const std::string scope : {"suffix", "middle", "equal"}) {
        DiscreteKeys subset;
        for (const auto& key : keys) {
          if (scope == "equal" ||
              (key.first > 0 && (scope == "suffix" || key.first + 1 < numberKeys)))
            subset.push_back(key);
        }
        // Compute the marginal independently, then construct the subset factor.
        std::vector<double> marginal(size_t(1) << subset.size(), 0);
        for (size_t index = 0; index < values.size(); ++index)
          marginal[projectIndex(index, numberKeys, subset)] += values[index];
        const TableFactor factor(subset, marginal);
        for (bool divide : {false, true}) {
          validate(divide ? joint / factor : joint * factor, values, marginal,
                   numberKeys, subset, divide);
          const auto samples = gtsam::timing::measureMilliseconds([&] {
            const TableFactor result = divide ? joint / factor : joint * factor;
            timingChecksum += result.nrValues();
          }, warmups, repetitions);
          const double microseconds = 1000 * gtsam::timing::summarizeSamples(
              samples, gtsam::timing::MedianPolicy::kUpperMiddle).median;
          const std::string operation = divide ? "divide" : "multiply";
          std::cout << numberKeys << ',' << sparse << ',' << scope << ','
                    << operation << ',' << microseconds << '\n';
          metrics.push_back({std::to_string(numberKeys) + "_keys_" +
                                 (sparse ? "sparse_" : "dense_") + scope + "_" +
                                 operation,
                             "microseconds", microseconds});
        }
      }
    }
  }
  if (!outputPath.empty())
    gtsam::timing::writeBenchmarkActionMetrics(outputPath, metrics);
  std::cerr << "timing checksum: " << timingChecksum << '\n';
  return 0;
}
