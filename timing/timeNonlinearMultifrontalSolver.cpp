/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeNonlinearMultifrontalSolver.cpp
 * @brief   Time NonlinearMultifrontalSolver on BAL 88-camera dataset.
 * @author  Frank Dellaert
 * @date    January 2026
 */

#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearMultifrontalSolver.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <optional>

#include "timeSFMBAL.h"

using namespace std;
using namespace gtsam;

namespace {
constexpr size_t kIterations = 2;
constexpr size_t kMergeDimCap = 16;
constexpr double kLambda = 1e7;
constexpr bool kDiagonalDamping = true;
constexpr double kMinDiagonal = 1e-6;
constexpr double kMaxDiagonal = 1e32;
}  // namespace

int main() {
  const string bal16 = findExampleDataFile("dubrovnik-16-22106-pre");
  // const string bal88 = findExampleDataFile("dubrovnik-88-64298-pre");
  // const string bal135 = findExampleDataFile("dubrovnik-135-90642-pre");
  for (const auto& filename : {bal16} /*, bal88, bal135*/) {
    cout << "\nProcessing BAL file: " << filename << std::endl;
    const SfmData db = SfmData::FromBalFile(filename);

    NonlinearFactorGraph graph = buildGeneralSfmGraph(db);
    Values values = buildGeneralSfmInitial(db);
    auto linear = *graph.linearize(values);

    auto orderings = createOrderings(db, linear);
    for (const auto& [label, ordering] : orderings) {
      cout << "\nBAL Benchmark (" << label << ", iterations=" << kIterations
           << "):" << std::endl;

      auto start = std::chrono::high_resolution_clock::now();
      NonlinearMultifrontalSolver::DampingParams dampingParams;
      dampingParams.diagonalDamping = kDiagonalDamping;
      dampingParams.minDiagonal = kMinDiagonal;
      dampingParams.maxDiagonal = kMaxDiagonal;
      MultifrontalSolver::Parameters mfParams;
      mfParams.mergeDimCap = kMergeDimCap;
      mfParams.qrMode = MultifrontalParameters::QRMode::Allow;
      NonlinearMultifrontalSolver solver(graph, values, ordering, mfParams,
                                         dampingParams);
      for (size_t i = 0; i < kIterations; ++i) {
        if (i > 0) linear = *graph.linearize(values);
        solver.eliminateInPlace(linear, kLambda);
        VectorValues delta = solver.updateSolution();
        values = values.retract(delta);
      }
      auto end = std::chrono::high_resolution_clock::now();

      std::chrono::duration<double> elapsed = end - start;
      cout << "Elapsed: " << elapsed.count() << " s" << std::endl;
    }
  }
  return 0;
}
