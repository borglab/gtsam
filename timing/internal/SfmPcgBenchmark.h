/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
