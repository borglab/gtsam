/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSfmPointBatchSchur.cpp
 * @brief Correctness tests for backend-neutral compact BAL Schur assembly.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/slam/dataset.h>

#include <Eigen/Cholesky>

#include "internal/SfmBalBenchmark.h"
#include "internal/SfmPointBatchSchur.h"

using namespace gtsam;
using namespace gtsam::timing::bal;
using symbol_shorthand::C;
using symbol_shorthand::P;

namespace {

GaussianFactorGraph tinyPointBatchSystem() {
  GaussianFactorGraph graph;
  auto batch = std::make_shared<PointBatchJacobian>(
      KeyVector{C(0), C(1), P(0)}, std::vector<size_t>{9, 9, 3});

  Matrix29 camera0, camera1;
  camera0 << 1.0, 0.2, -0.1, 0.3, 0.0, 0.4, -0.2, 0.1, 0.5,
      -0.3, 0.8, 0.2, 0.0, 0.6, -0.1, 0.3, 0.2, -0.4;
  camera1 << 0.4, -0.2, 0.5, 0.1, -0.3, 0.7, 0.2, 0.6, -0.1,
      0.0, 0.3, -0.4, 0.8, 0.2, 0.1, -0.5, 0.7, 0.4;
  Matrix23 point0, point1;
  point0 << 0.7, -0.3, 0.2, 0.1, 0.6, -0.5;
  point1 << -0.2, 0.4, 0.8, 0.5, -0.1, 0.3;
  const Vector2 rhs0(0.6, -0.4), rhs1(-0.2, 0.7);
  batch->addRow({0, 2}, {camera0, point0}, rhs0);
  batch->addRow({1, 2}, {camera1, point1}, rhs1);
  graph.push_back(batch);

  const Matrix99 cameraPrior0 = 1.7 * Matrix99::Identity();
  const Matrix99 cameraPrior1 = 1.3 * Matrix99::Identity();
  const Vector9 cameraRhs0 =
      (Vector9() << 0.1, -0.2, 0.3, 0.0, 0.2, -0.1, 0.4, 0.1, -0.3)
          .finished();
  const Vector9 cameraRhs1 =
      (Vector9() << -0.2, 0.1, 0.0, 0.3, -0.1, 0.2, -0.4, 0.2, 0.1)
          .finished();
  graph.emplace_shared<JacobianFactor>(C(0), cameraPrior0, cameraRhs0);
  graph.emplace_shared<JacobianFactor>(C(1), cameraPrior1, cameraRhs1);

  const Matrix3 pointPrior = 1.1 * Matrix3::Identity();
  const Vector3 pointRhs(0.2, -0.1, 0.3);
  graph.emplace_shared<JacobianFactor>(P(0), pointPrior, pointRhs);
  return graph;
}

Matrix denseCameraMatrix(const CompactCameraSystem& system) {
  Matrix result = Matrix::Zero(9 * system.cameraCount,
                               9 * system.cameraCount);
  for (size_t row = 0; row < system.cameraCount; ++row) {
    for (size_t column = row; column < system.cameraCount; ++column) {
      const size_t slot =
          upperCameraBlockIndex(row, column, system.cameraCount);
      if (!system.usedBlocks[slot]) continue;
      result.block<9, 9>(9 * row, 9 * column) = system.blocks[slot];
      if (row != column) {
        result.block<9, 9>(9 * column, 9 * row) =
            system.blocks[slot].transpose();
      }
    }
  }
  return result;
}

void verifyAgainstDenseReference(const GaussianFactorGraph& graph,
                                 size_t cameraCount, size_t pointCount,
                                 double tolerance, TestResult& result_,
                                 const std::string& name_) {
  Ordering ordering;
  for (size_t camera = 0; camera < cameraCount; ++camera) {
    ordering.push_back(C(camera));
  }
  for (size_t point = 0; point < pointCount; ++point) {
    ordering.push_back(P(point));
  }
  const auto [fullHessian, fullRhs] = graph.hessian(ordering);

  const DenseIndex cameraDimension = 9 * cameraCount;
  const DenseIndex pointDimension = 3 * pointCount;
  const Matrix cameraHessian =
      fullHessian.topLeftCorner(cameraDimension, cameraDimension);
  const Matrix cameraPoint = fullHessian.block(
      0, cameraDimension, cameraDimension, pointDimension);
  const Matrix pointHessian = fullHessian.bottomRightCorner(
      pointDimension, pointDimension);
  const Matrix pointCovariance = pointHessian.inverse();
  const Matrix expectedReducedHessian =
      cameraHessian - cameraPoint * pointCovariance * cameraPoint.transpose();
  const Vector expectedReducedRhs =
      fullRhs.head(cameraDimension) -
      cameraPoint * pointCovariance * fullRhs.tail(pointDimension);

  const CompactCameraSystem compact =
      buildPointBatchCameraSystemParallel(graph);
  EXPECT_LONGS_EQUAL(cameraCount, compact.cameraCount);
  EXPECT(assert_equal(expectedReducedHessian, denseCameraMatrix(compact),
                      tolerance));
  EXPECT(assert_equal(expectedReducedRhs, compact.rhs, tolerance));

  const Vector cameraDelta =
      expectedReducedHessian.llt().solve(expectedReducedRhs);
  VectorValues cameraSolution;
  for (size_t camera = 0; camera < cameraCount; ++camera) {
    cameraSolution.insert(C(camera), cameraDelta.segment<9>(9 * camera));
  }
  const VectorValues compactSolution =
      backSubstitutePointBatchLandmarksParallel(compact, cameraSolution);
  const Vector reference = fullHessian.llt().solve(fullRhs);
  for (size_t camera = 0; camera < cameraCount; ++camera) {
    EXPECT(assert_equal(reference.segment<9>(9 * camera),
                        compactSolution.at(C(camera)), tolerance));
  }
  for (size_t point = 0; point < pointCount; ++point) {
    EXPECT(assert_equal(
        reference.segment<3>(cameraDimension + 3 * point),
        compactSolution.at(P(point)), tolerance));
  }
}

}  // namespace

// Dense matrices here are deliberately confined to this tiny correctness test.
TEST(SfmPointBatchSchur, MatchesGenericReductionAndBackSubstitution) {
  const GaussianFactorGraph graph = tinyPointBatchSystem();
  verifyAgainstDenseReference(graph, 2, 1, 1e-10, result_, name_);
}

/* ************************************************************************* */
namespace tiny_bal_fixture {

GaussianFactorGraph createDampedSystem(const SfmData& data) {
  const BalBenchmarkConfig config;
  const NonlinearFactorGraph nonlinear =
      buildBatchSfmGraph(data, config, false, 0);
  const Values initial = buildGeneralSfmInitial(data);
  GaussianFactorGraph damped = *nonlinear.linearize(initial);

  const Matrix99 cameraDamping = 0.2 * Matrix99::Identity();
  const Matrix3 pointDamping = 0.2 * Matrix3::Identity();
  for (size_t camera = 0; camera < data.numberCameras(); ++camera) {
    damped.emplace_shared<JacobianFactor>(C(camera), cameraDamping,
                                          Vector9::Zero());
  }
  for (size_t point = 0; point < data.numberTracks(); ++point) {
    damped.emplace_shared<JacobianFactor>(P(point), pointDamping,
                                          Vector3::Zero());
  }
  return damped;
}

// Exercise the production batch linearization on the repository's tiny BAL
// fixture. Dense reduction remains confined to this correctness test.
TEST(SfmPointBatchSchur, TinyBalFixtureMatchesGenericReduction) {
  const SfmData data =
      loadDataset(findExampleDataFile("dubrovnik-3-7-pre"));
  const GaussianFactorGraph damped = createDampedSystem(data);

  verifyAgainstDenseReference(damped, data.numberCameras(),
                              data.numberTracks(), 1e-6, result_, name_);
}

// Verify partial multifrontal point elimination produces the same reduced
// camera system as the compact SFM Schur implementation.
TEST(SfmPointBatchSchur, PartialMultifrontalMatchesReducedCameraSystem) {
  const SfmData data =
      loadDataset(findExampleDataFile("dubrovnik-3-7-pre"));
  const GaussianFactorGraph damped = createDampedSystem(data);
  const CompactCameraSystem reducedCameraSystem =
      buildPointBatchCameraSystemParallel(damped);

  const Ordering pointFirstOrdering = createSchurOrdering(data, false);
  MultifrontalSolver solver(damped, pointFirstOrdering,
                            data.numberTracks());
  solver.eliminatePartialInPlace(damped);
  const GaussianFactorGraph remaining = solver.remainingFactorGraph();

  Ordering cameraOrdering;
  for (size_t camera = 0; camera < data.numberCameras(); ++camera) {
    cameraOrdering.push_back(C(camera));
  }
  const auto [actualHessian, actualRhs] = remaining.hessian(cameraOrdering);
  EXPECT(assert_equal(denseCameraMatrix(reducedCameraSystem), actualHessian,
                      1e-6));
  EXPECT(assert_equal(reducedCameraSystem.rhs, actualRhs, 1e-6));
}

}  // namespace tiny_bal_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
