/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MatrixWeightedLocalizationExample.cpp
 * @brief Locally solve a noisy matrix-weighted localization problem.
 * @author Avinash Subramanian
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <iostream>

#include "MatrixWeightedLocalizationUtils.h"

using namespace gtsam;
using namespace gtsam::examples;

int main() {
  const auto problem = loadMatrixWeightedLocalization(
      findExampleDataFile("matrix_weighted_localization.g2o"));

  GaussNewtonParams parameters;
  parameters.setMaxIterations(100);
  parameters.setRelativeErrorTol(1e-12);
  const Values result_kTws =
      GaussNewtonOptimizer(problem.graph, problem.initial_kTws, parameters)
          .optimize();

  const double groundTruthError = problem.graph.error(problem.groundTruth_kTws);
  const double initialError = problem.graph.error(problem.initial_kTws);
  const double finalError = problem.graph.error(result_kTws);
  const auto poseErrors = poseErrorStatistics(problem.groundTruth_kTws,
                                              result_kTws, problem.poseKeys);

  std::cout << "Ground-truth error: " << groundTruthError << '\n'
            << "Initial error: " << initialError << '\n'
            << "Final error: " << finalError << '\n'
            << "Mean pose error: " << poseErrors.meanPoseError << '\n'
            << "Maximum pose error: " << poseErrors.maximumPoseError
            << std::endl;

  constexpr double kMaximumPoseError = 0.05;
  if (finalError >= initialError || finalError >= groundTruthError ||
      poseErrors.maximumPoseError >= kMaximumPoseError) {
    std::cerr << "Matrix-weighted localization did not improve the noisy "
                 "problem as expected."
              << std::endl;
    return 1;
  }

  return 0;
}
