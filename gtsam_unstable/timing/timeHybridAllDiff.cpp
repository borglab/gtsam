/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file timeHybridAllDiff.cpp
 * @brief Benchmark sparse AllDiff multiplication and table dispatch.
 */

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/discrete/AllDiff.h>

#include <Eigen/Sparse>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"

namespace {

using gtsam::AllDiff;
using gtsam::DecisionTreeFactor;
using gtsam::DiscreteFactor;
using gtsam::DiscreteKey;
using gtsam::DiscreteKeys;
using gtsam::TableFactor;
using gtsam::symbol_shorthand::A;
using gtsam::timing::BenchmarkMetric;
using gtsam::timing::MedianPolicy;
using gtsam::timing::TimingSummary;

volatile uint64_t timingChecksum = 0;

struct BenchmarkOptions {
  size_t minimumObjects = 3;
  size_t maximumObjects = 7;
  size_t warmups = 1;
  size_t repetitions = 5;
  std::optional<std::string> outputPath;
};

struct ProductTimings {
  std::vector<double> decisionTree;
  std::vector<double> copiedTable;
  std::vector<double> directTable;
};

DiscreteKeys makeKeys(size_t numberObjects) {
  DiscreteKeys keys;
  keys.reserve(numberObjects);
  for (size_t i = 0; i < numberObjects; ++i) {
    keys.emplace_back(A(i), numberObjects);
  }
  return keys;
}

TableFactor makeDenseTable(const DiscreteKeys& keys) {
  uint64_t cardinality = 1;
  for (const DiscreteKey& key : keys) cardinality *= key.second;

  Eigen::SparseVector<double> values(cardinality);
  values.reserve(cardinality);
  for (uint64_t index = 0; index < cardinality; ++index) {
    values.insert(index) = 1.0 + static_cast<double>(index % 17) / 17.0;
  }
  return TableFactor(keys, values);
}

// Reproduce the decision-tree conversion used before sparse AllDiff support.
DiscreteFactor::shared_ptr decisionTreeProduct(
    const TableFactor& input, const std::shared_ptr<AllDiff>& constraint) {
  const DecisionTreeFactor product =
      constraint->operator*(input.toDecisionTreeFactor());
  return std::make_shared<TableFactor>(product);
}

// Reproduce the former generic TableFactor dispatch: copy the input table into
// an owning shared pointer, then ask the other factor to multiply it.
DiscreteFactor::shared_ptr copiedTableProduct(
    const TableFactor& input, const std::shared_ptr<AllDiff>& constraint) {
  const auto copiedInput = std::make_shared<TableFactor>(input);
  return constraint->multiply(copiedInput);
}

// Use the current dispatch, which borrows the input and converts AllDiff.
DiscreteFactor::shared_ptr directTableProduct(
    const TableFactor& input, const std::shared_ptr<AllDiff>& constraint) {
  return input.multiply(constraint);
}

template <class Callable>
double timeBatchMicroseconds(Callable&& callable, size_t operations) {
  const double seconds = gtsam::timing::measureSeconds([&] {
    for (size_t i = 0; i < operations; ++i) {
      const DiscreteFactor::shared_ptr product = callable();
      timingChecksum += product->nrValues();
    }
  });
  return seconds * 1e6 / static_cast<double>(operations);
}

size_t operationsFor(size_t numberObjects) {
  if (numberObjects == 3) return 200;
  if (numberObjects == 4) return 30;
  if (numberObjects == 5) return 3;
  return 1;
}

ProductTimings measureProducts(const TableFactor& input,
                               const std::shared_ptr<AllDiff>& constraint,
                               const BenchmarkOptions& options) {
  const size_t operations = operationsFor(constraint->size());
  const auto decisionTree = [&] {
    return decisionTreeProduct(input, constraint);
  };
  const auto copiedTable = [&] {
    return copiedTableProduct(input, constraint);
  };
  const auto directTable = [&] {
    return directTableProduct(input, constraint);
  };

  for (size_t i = 0; i < options.warmups; ++i) {
    static_cast<void>(timeBatchMicroseconds(decisionTree, operations));
    static_cast<void>(timeBatchMicroseconds(copiedTable, operations));
    static_cast<void>(timeBatchMicroseconds(directTable, operations));
  }

  ProductTimings timings;
  timings.decisionTree.reserve(options.repetitions);
  timings.copiedTable.reserve(options.repetitions);
  timings.directTable.reserve(options.repetitions);
  for (size_t repetition = 0; repetition < options.repetitions; ++repetition) {
    // Rotate the order to reduce systematic CPU-frequency and thermal bias.
    for (size_t offset = 0; offset < 3; ++offset) {
      switch ((repetition + offset) % 3) {
        case 0:
          timings.decisionTree.push_back(
              timeBatchMicroseconds(decisionTree, operations));
          break;
        case 1:
          timings.copiedTable.push_back(
              timeBatchMicroseconds(copiedTable, operations));
          break;
        case 2:
          timings.directTable.push_back(
              timeBatchMicroseconds(directTable, operations));
          break;
      }
    }
  }
  return timings;
}

TimingSummary summarize(const std::vector<double>& samples) {
  return gtsam::timing::summarizeSamples(samples, MedianPolicy::kUpperMiddle);
}

void verifyProducts(const TableFactor& input,
                    const std::shared_ptr<AllDiff>& constraint) {
  const TableFactor expected =
      directTableProduct(input, constraint)->toTableFactor();
  for (const DiscreteFactor::shared_ptr& product :
       {decisionTreeProduct(input, constraint),
        copiedTableProduct(input, constraint)}) {
    if (!expected.equals(product->toTableFactor(), 1e-9)) {
      throw std::runtime_error("AllDiff benchmark products disagree");
    }
  }
}

void printUsage() {
  std::cout << "Usage: timeHybridAllDiff [--minimum-objects N] "
               "[--maximum-objects N] [--warmup N] [--repeats N] "
               "[--output FILE]\n";
}

}  // namespace

int main(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  if (arguments.helpRequested()) {
    printUsage();
    return 0;
  }

  const std::string outputPath = arguments.stringValue("--output", "");
  const BenchmarkOptions options{
      arguments.sizeValue("--minimum-objects", 3),
      arguments.sizeValue("--maximum-objects", 7),
      arguments.sizeValue("--warmup", 1),
      arguments.sizeValue("--repeats", 5),
      outputPath.empty() ? std::optional<std::string>() : outputPath,
  };
  arguments.validateAllConsumed();
  if (options.minimumObjects < 2 ||
      options.maximumObjects < options.minimumObjects ||
      options.maximumObjects > 7 || options.repetitions == 0) {
    throw std::invalid_argument(
        "require 2 <= minimum <= maximum <= 7 and repeats >= 1");
  }

  std::cout
      << "decision_tree: pre-sparse conversion baseline\n"
      << "copied_table: former generic dispatch, including a copy of the "
         "input table\n"
      << "direct_table: current virtual conversion dispatch, borrowing the "
         "input table\n\n"
      << "scope objects input_nonzeros result_nonzeros decision_tree_us "
         "copied_table_us direct_table_us tree_speedup copy_speedup\n";

  std::vector<BenchmarkMetric> metrics;
  for (const bool partialScope : {false, true}) {
    const std::string scope = partialScope ? "partial" : "full";
    for (size_t numberObjects = options.minimumObjects;
         numberObjects <= options.maximumObjects; ++numberObjects) {
      const DiscreteKeys constraintKeys = makeKeys(numberObjects);
      const size_t inputKeyCount =
          partialScope ? (numberObjects + 1) / 2 : numberObjects;
      const DiscreteKeys inputKeys(constraintKeys.begin(),
                                   constraintKeys.begin() + inputKeyCount);
      const TableFactor input = makeDenseTable(inputKeys);
      const auto constraint = std::make_shared<AllDiff>(constraintKeys);
      verifyProducts(input, constraint);

      const uint64_t resultNonzeros =
          directTableProduct(input, constraint)->nrValues();
      const ProductTimings samples =
          measureProducts(input, constraint, options);
      const double decisionTree = summarize(samples.decisionTree).median;
      const double copiedTable = summarize(samples.copiedTable).median;
      const double directTable = summarize(samples.directTable).median;

      std::cout << std::fixed << std::setprecision(2) << scope << ' '
                << numberObjects << ' ' << input.nrValues() << ' '
                << resultNonzeros << ' ' << decisionTree << ' ' << copiedTable
                << ' ' << directTable << ' ' << decisionTree / directTable
                << ' ' << copiedTable / directTable << '\n';

      const std::string prefix =
          scope + "_" + std::to_string(numberObjects) + "_objects_";
      metrics.push_back(
          {prefix + "decision_tree", "microseconds", decisionTree});
      metrics.push_back({prefix + "copied_table", "microseconds", copiedTable});
      metrics.push_back({prefix + "direct_table", "microseconds", directTable});
      metrics.push_back(
          {prefix + "tree_speedup", "x", decisionTree / directTable});
      metrics.push_back(
          {prefix + "copy_speedup", "x", copiedTable / directTable});
    }
  }

  if (options.outputPath) {
    gtsam::timing::writeBenchmarkActionMetrics(*options.outputPath, metrics);
  }
  std::cerr << "timing checksum: " << timingChecksum << '\n';
  return 0;
}
