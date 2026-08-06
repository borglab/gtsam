/**
 * @file SfmPcgBenchmark.h
 * @brief Private end-to-end BAL PCG benchmark interface.
 */

#pragma once

#include <string>
#include <vector>

#include "SfmBalBenchmark.h"

namespace gtsam::timing::bal {

/**
 * Compare direct, full-system PCG, and reduced-camera PCG BAL optimization.
 */
void runEndToEndPcgComparison(const std::vector<std::string>& filenames,
                              const BalBenchmarkConfig& config);

}  // namespace gtsam::timing::bal
