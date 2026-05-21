/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRiemannianStaircase.cpp
 * @brief   Unit tests for the RiemannianStaircaseOptimizer helpers.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/RiemannianStaircaseOptimizer.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <cmath>
#include <iostream>

using namespace gtsam;

namespace RingFixture {

NonlinearFactorGraph RingGraph(size_t numPoses, double delta) {
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < numPoses; ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        Symbol('x', i), Symbol('x', (i + 1) % numPoses),
        Rot2::fromAngle(delta));
  }
  return graph;
}

// Matrix-form (D=2) Stiefel fixture: each value is a 2x2 row-orthonormal
// matrix. This is the base of the Riemannian Staircase ladder for Rot2.
Values RingQcqpValuesD2(size_t numPoses, double delta, double perturbation) {
  Values values;
  for (size_t i = 0; i < numPoses; ++i) {
    InsertQcqpValue<Rot2, 2>(
        Symbol('x', i),
        Rot2::fromAngle(i * delta + perturbation * static_cast<double>(i)),
        &values);
  }
  return values;
}

}  // namespace RingFixture

/* ************************************************************************* */
// Verifies that the multipliers ALM converges to, when plugged into our
// assembled certificate S = Q + sum_m lambda_m * (A_m + A_m^T), satisfy KKT
// stationarity S * Y* ~= 0 and yield a positive-(semi)definite S at a good
// minimum of the 5-Rot2 ring (we expect min eig(S) >= -eta). Run at the
// natural matrix form K=2 (the base of the staircase ladder).
TEST(RiemannianStaircase, CertificateMatchesKKT) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);
  constexpr size_t K = 2;

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const QcqpProblem qcqp(graph, K);
  const Values initialValues = RingQcqpValuesD2(N, delta, 0.01);

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->storeOptProgress = true;
  params->maxIterations = 100;
  params->initialMuEq = 10.0;
  params->muEqIncreaseRate = 2.0;
  params->absoluteViolationTolerance = 1e-10;
  params->relativeViolationTolerance = 1e-10;
  params->absoluteCostTolerance = 1e-12;
  params->relativeCostTolerance = 1e-12;
  params->verbose = false;

  AugmentedLagrangianOptimizer alm(qcqp, initialValues, params);
  const Values Ystar = alm.optimize();

  EXPECT(!alm.progress().empty());
  const auto& lambdaEq = alm.progress().back().lambdaEq;
  EXPECT_LONGS_EQUAL(static_cast<long>(qcqp.eConstraints().size()),
                     static_cast<long>(lambdaEq.size()));

  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);
  const size_t totalDim = layout.totalDim;
  LONGS_EQUAL(N * 2, totalDim);  // row dim = 2 per pose

  const Eigen::SparseMatrix<double> S =
      RiemannianStaircaseOptimizer::assembleCertificate(qcqp, layout, lambdaEq);
  LONGS_EQUAL(static_cast<long>(totalDim), S.rows());
  LONGS_EQUAL(static_cast<long>(totalDim), S.cols());

  // Stack Y* into a single (totalDim x K) matrix via the Layout helper.
  const Matrix Ymat = layout.stack(Ystar);

  // KKT stationarity: S * Y* should be approximately zero.
  const Matrix SY = S * Ymat;
  const double stationarityRatio = SY.norm() / Ymat.norm();
  std::cout << "||S Y|| / ||Y|| = " << stationarityRatio << std::endl;
  EXPECT(stationarityRatio < 1e-3);

  // Min eigenvalue >= -eta indicates a certifiable (near-PSD) certificate.
  // Cross-check via dense Eigen here (10x10 only; production path uses LOBPCG
  // via fast_verification).
  const Matrix Sdense(S);
  Eigen::SelfAdjointEigenSolver<Matrix> es(Sdense);
  const double lambdaMin = es.eigenvalues().minCoeff();
  std::cout << "min eig(S) = " << lambdaMin << std::endl;
  EXPECT(lambdaMin > -1e-3);
}

/* ************************************************************************* */
// Verifies escapeSaddleAndLift produces values one column wider than Ystar,
// leaves the first p columns unchanged, and writes the requested descent
// direction (alpha * v) slice into the new column.
TEST(RiemannianStaircase, EscapeSaddleAndLiftShape) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  // K=2 representation: Y_i is 2x2 row-orthonormal at the base of the ladder.
  const Values Ystar = RingQcqpValuesD2(N, delta, 0.0);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);
  LONGS_EQUAL(N * 2, layout.totalDim);  // 2 rows per pose

  // Construct a deterministic descent direction with known entries.
  Vector v =
      Vector::LinSpaced(static_cast<int>(layout.totalDim), 1.0, layout.totalDim);
  v.normalize();

  const double alpha = 1e-2;
  const Values Ynext =
      RiemannianStaircaseOptimizer::escapeSaddleAndLift(Ystar, v, layout, alpha);

  // (1) Same key set, each entry has one more column.
  EXPECT_LONGS_EQUAL(static_cast<long>(Ystar.size()),
                     static_cast<long>(Ynext.size()));
  for (const auto& [key, Yi] : Ystar.extract<Matrix>()) {
    const Matrix YiNew = Ynext.at<Matrix>(key);
    EXPECT_LONGS_EQUAL(Yi.rows(), YiNew.rows());
    EXPECT_LONGS_EQUAL(Yi.cols() + 1, YiNew.cols());

    // (2) Left columns unchanged.
    EXPECT(assert_equal(Matrix(Yi), Matrix(YiNew.leftCols(Yi.cols())), 1e-15));

    // (3) New column equals alpha * v slice.
    const auto& slice = layout.sliceOf(key);
    const Matrix expectedCol = alpha * v.segment(slice.offset, slice.rowDim);
    EXPECT(assert_equal(expectedCol,
                        Matrix(YiNew.rightCols(1)), 1e-15));
  }
}

/* ************************************************************************* */
// Verifies the constraint violation at the lifted point grows like O(alpha^2):
// for a feasible Ystar at rank p with constraints h_m(Y) = trace(Y' A_m Y) - b
// = 0, lifting to [Ystar | alpha v] gives h_m = alpha^2 * v' A_m v. So
// doubling alpha must quadruple the violation norm (to leading order).
TEST(RiemannianStaircase, EscapeSaddleViolationIsQuadratic) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  // Matrix-form (K=2) representation so the lift from K=2 to K=3 is a legal
  // step of the staircase ladder.
  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values Ystar = RingQcqpValuesD2(N, delta, 0.0);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(Ystar);
  const size_t totalDim = layout.totalDim;

  // Sanity: Ystar is feasible at the base of the ladder (K=2).
  const QcqpProblem qcqpBase(graph, /*K=*/2);
  EXPECT_DOUBLES_EQUAL(0.0, qcqpBase.eConstraints().violationNorm(Ystar),
                       1e-12);

  Vector v = Vector::LinSpaced(static_cast<int>(totalDim), 1.0, totalDim);
  v.normalize();

  const QcqpProblem qcqpLifted(graph, /*K=*/3);
  const double alphaSmall = 1e-3;
  const double alphaLarge = 2e-3;
  const Values Yh = RiemannianStaircaseOptimizer::escapeSaddleAndLift(
      Ystar, v, layout, alphaSmall);
  const Values YH = RiemannianStaircaseOptimizer::escapeSaddleAndLift(
      Ystar, v, layout, alphaLarge);

  const double vioSmall = qcqpLifted.eConstraints().violationNorm(Yh);
  const double vioLarge = qcqpLifted.eConstraints().violationNorm(YH);
  std::cout << "vio(alpha=" << alphaSmall << ") = " << vioSmall << std::endl;
  std::cout << "vio(alpha=" << alphaLarge << ") = " << vioLarge << std::endl;

  // Violation should be strictly positive (lift breaks feasibility).
  EXPECT(vioSmall > 0.0);
  // Doubling alpha should quadruple the violation; check to within 1% of the
  // expected factor of 4 since higher-order terms are negligible at this
  // scale.
  EXPECT_DOUBLES_EQUAL(4.0, vioLarge / vioSmall, 4e-2);
}

/* ************************************************************************* */
// Cross-validation: the LOBPCG and DenseEigen verification paths should
// reach the same verdict and produce comparable lambda_min estimates on a
// problem that fits in dense memory. If this disagrees, one of the two paths
// has a bug -- most often a misaligned offset in assembleCertificate, which
// dense Eigen would see directly but LOBPCG might not.
TEST(RiemannianStaircase, LOBPCGAndDenseAgree) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values initial = RingQcqpValuesD2(N, delta, /*perturbation=*/0.01);

  RiemannianStaircaseParams paramsL;
  paramsL.pMin = 2;
  paramsL.pMax = 5;
  paramsL.alpha = 1e-2;
  paramsL.eta = 1e-3;
  paramsL.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::LOBPCG;
  paramsL.almParams->maxIterations = 100;
  paramsL.almParams->initialMuEq = 10.0;
  paramsL.almParams->muEqIncreaseRate = 2.0;
  paramsL.almParams->absoluteViolationTolerance = 1e-8;
  paramsL.almParams->relativeViolationTolerance = 1e-8;

  RiemannianStaircaseParams paramsD = paramsL;
  paramsD.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::DenseEigen;
  // Use a fresh almParams shared_ptr so the two runs don't share state.
  paramsD.almParams =
      std::make_shared<AugmentedLagrangianParams>(*paramsL.almParams);

  QcqpProblem qcqpL(graph, paramsL.pMin);
  QcqpProblem qcqpD(graph, paramsD.pMin);
  const auto resultL =
      RiemannianStaircaseOptimizer(qcqpL, initial, paramsL).optimize();
  const auto resultD =
      RiemannianStaircaseOptimizer(qcqpD, initial, paramsD).optimize();

  std::cout << "[XV] LOBPCG  certified=" << resultL.certified
            << " finalRank=" << resultL.finalRank
            << " minEig=" << resultL.minEigenvalue << std::endl;
  std::cout << "[XV] Dense   certified=" << resultD.certified
            << " finalRank=" << resultD.finalRank
            << " minEig=" << resultD.minEigenvalue << std::endl;

  EXPECT(resultL.certified == resultD.certified);
  EXPECT_LONGS_EQUAL(static_cast<long>(resultL.finalRank),
                     static_cast<long>(resultD.finalRank));
  // LOBPCG and dense eigenvalues should agree to within LOBPCG's tolerance.
  // The LOBPCG path returns 0 when the Cholesky succeeded; the dense path
  // always returns the true min eigenvalue. Only compare when LOBPCG actually
  // computed a Ritz value (i.e. when the certificate failed).
  if (!resultL.certified) {
    EXPECT(std::abs(resultL.minEigenvalue - resultD.minEigenvalue) < 1e-2);
  }
}

/* ************************************************************************* */
// Cross-validation: the Spectra-based Lanczos path should match DenseEigen on
// both verdict and lambda_min. Unlike LOBPCG, Spectra always reports the true
// Ritz value (no early-exit shortcut), so we can compare lambda_min on both
// certified and uncertified runs.
TEST(RiemannianStaircase, SpectraAndDenseAgree) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values initial = RingQcqpValuesD2(N, delta, /*perturbation=*/0.01);

  RiemannianStaircaseParams paramsS;
  paramsS.pMin = 2;
  paramsS.pMax = 5;
  paramsS.alpha = 1e-2;
  paramsS.eta = 1e-3;
  paramsS.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::Spectra;
  paramsS.almParams->maxIterations = 100;
  paramsS.almParams->initialMuEq = 10.0;
  paramsS.almParams->muEqIncreaseRate = 2.0;
  paramsS.almParams->absoluteViolationTolerance = 1e-8;
  paramsS.almParams->relativeViolationTolerance = 1e-8;

  RiemannianStaircaseParams paramsD = paramsS;
  paramsD.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::DenseEigen;
  paramsD.almParams =
      std::make_shared<AugmentedLagrangianParams>(*paramsS.almParams);

  QcqpProblem qcqpS(graph, paramsS.pMin);
  QcqpProblem qcqpD(graph, paramsD.pMin);
  const auto resultS =
      RiemannianStaircaseOptimizer(qcqpS, initial, paramsS).optimize();
  const auto resultD =
      RiemannianStaircaseOptimizer(qcqpD, initial, paramsD).optimize();

  std::cout << "[XV] Spectra certified=" << resultS.certified
            << " finalRank=" << resultS.finalRank
            << " minEig=" << resultS.minEigenvalue << std::endl;
  std::cout << "[XV] Dense   certified=" << resultD.certified
            << " finalRank=" << resultD.finalRank
            << " minEig=" << resultD.minEigenvalue << std::endl;

  EXPECT(resultS.certified == resultD.certified);
  EXPECT_LONGS_EQUAL(static_cast<long>(resultS.finalRank),
                     static_cast<long>(resultD.finalRank));
  // Spectra computes a true Ritz value at every level, so lambda_min should
  // agree with the dense path to within Lanczos tolerance.
  EXPECT(std::abs(resultS.minEigenvalue - resultD.minEigenvalue) < 1e-3);
}

/* ************************************************************************* */
// Easy case: well-initialized 5-Rot2 ring. The relaxation is tight at the
// starting rank; certificate should pass at p=1 (or at worst p=2).
TEST(RiemannianStaircase, Rot2RingEasy) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  const Values initial = RingQcqpValuesD2(N, delta, /*perturbation=*/0.01);

  RiemannianStaircaseParams params;
  params.pMin = 2;
  params.pMax = 5;
  params.alpha = 1e-2;
  params.eta = 1e-3;
  params.almParams->maxIterations = 100;
  params.almParams->initialMuEq = 10.0;
  params.almParams->muEqIncreaseRate = 2.0;
  params.almParams->absoluteViolationTolerance = 1e-8;
  params.almParams->relativeViolationTolerance = 1e-8;
  params.almParams->absoluteCostTolerance = 1e-10;
  params.almParams->relativeCostTolerance = 1e-10;

  QcqpProblem qcqp(graph, params.pMin);
  const RiemannianStaircaseOptimizer rso(qcqp, initial, params);
  const RiemannianStaircaseResult result = rso.optimize();

  EXPECT(result.certified);
  EXPECT(result.finalRank <= 3);  // D=2 ladder starts at p=2; allow one lift
  EXPECT(result.minEigenvalue > -params.eta);
}

/* ************************************************************************* */
// End-to-end check with a "wrong-winding" initialization: staircase reaches
// a certified solution and costs are non-increasing across levels.
TEST(RiemannianStaircase, Rot2RingHardEscape) {
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingFixture::RingGraph(N, delta);

  // Bad init in matrix form K=2 (row-orthonormal R^T): -i*delta winds opposite
  // of the measurements; ALM may settle in a wrong-winding local minimum from
  // which the staircase has to lift to escape.
  Values initial;
  for (size_t i = 0; i < N; ++i) {
    InsertQcqpValue<Rot2, 2>(Symbol('x', i),
                             Rot2::fromAngle(-static_cast<double>(i) * delta),
                             &initial);
  }

  RiemannianStaircaseParams params;
  params.pMin = 2;
  params.pMax = 5;
  params.alpha = 1e-1;
  params.eta = 1e-3;
  params.verbose = false;
  params.almParams->maxIterations = 200;
  params.almParams->initialMuEq = 10.0;
  params.almParams->muEqIncreaseRate = 2.0;
  params.almParams->absoluteViolationTolerance = 1e-8;
  params.almParams->relativeViolationTolerance = 1e-8;
  params.almParams->absoluteCostTolerance = 1e-10;
  params.almParams->relativeCostTolerance = 1e-10;

  QcqpProblem qcqp(graph, params.pMin);
  const RiemannianStaircaseOptimizer rso(qcqp, initial, params);
  const RiemannianStaircaseResult result = rso.optimize();

  std::cout << "[Hard] certified=" << result.certified
            << " finalRank=" << result.finalRank
            << " ranksVisited=" << result.ranksVisited.size() << std::endl;
  for (size_t i = 0; i < result.costPerLevel.size(); ++i) {
    std::cout << "  cost at p=" << result.ranksVisited[i] << ": "
              << result.costPerLevel[i] << std::endl;
  }

  EXPECT(result.certified);
  // Costs non-increasing across levels (modulo numerical tol).
  for (size_t i = 1; i < result.costPerLevel.size(); ++i) {
    EXPECT(result.costPerLevel[i] <= result.costPerLevel[i - 1] + 1e-6);
  }
}

/* ************************************************************************* */
// Sanity check: Layout::From walks variables in canonical (sorted-Key) order,
// assigns contiguous offsets, and totalDim equals the sum of row dims.
TEST(RiemannianStaircase, LayoutOrderAndTotalDim) {
  Values values;
  const Matrix zero4x3 = Matrix::Zero(4, 3);
  // Insert in non-sorted-Key order to ensure Layout::From sorts.
  values.insert(Symbol('x', 2), zero4x3);
  values.insert(Symbol('x', 0), zero4x3);
  values.insert(Symbol('x', 1), zero4x3);

  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);

  LONGS_EQUAL(12, layout.totalDim);
  LONGS_EQUAL(3, layout.size());
  LONGS_EQUAL(0, layout.offsetOf(Symbol('x', 0)));
  LONGS_EQUAL(4, layout.offsetOf(Symbol('x', 1)));
  LONGS_EQUAL(8, layout.offsetOf(Symbol('x', 2)));
  LONGS_EQUAL(4, layout.rowDimOf(Symbol('x', 0)));
  LONGS_EQUAL(4, layout.rowDimOf(Symbol('x', 1)));
  LONGS_EQUAL(4, layout.rowDimOf(Symbol('x', 2)));
}

/* ************************************************************************* */
// pMin/pMax sanity is the only configuration check in validateParams; verify
// pMin > pMax is rejected at construction.
TEST(RiemannianStaircase, RejectsInvertedPMinPMax) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
      Symbol('x', 0), Symbol('x', 1), Rot2::fromAngle(0.3));
  Values values;
  InsertQcqpValue<Rot2, 2>(Symbol('x', 0), Rot2::fromAngle(0.0), &values);
  InsertQcqpValue<Rot2, 2>(Symbol('x', 1), Rot2::fromAngle(0.0), &values);

  RiemannianStaircaseParams params;
  params.pMin = 3;
  params.pMax = 2;  // pMin > pMax -- must throw
  QcqpProblem qcqp(graph, params.pMin);
  CHECK_EXCEPTION(
      { RiemannianStaircaseOptimizer rso(qcqp, values, params); },
      std::invalid_argument);
}

/* ************************************************************************* */
// Layout struct tests: focused coverage of the
// RiemannianStaircaseOptimizer::Layout type (offsetOf / rowDimOf / sliceOf /
// conformsTo / stack / unstack) including heterogeneous row dims for keys
// of different rotation types (e.g., Rot2 + Rot3 in one Values).

using Layout = RiemannianStaircaseOptimizer::Layout;

TEST(Layout, HeterogeneousRowDims) {
  // Build a Values with mixed row dims to mimic a hypothetical Rot2 + Rot3
  // graph: x0 is a Rot2 matrix-form variable (2 rows), x1 is a Rot3 one (3
  // rows), x2 another Rot2 (2 rows).
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 2)));
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(3, 2)));
  values.insert(Symbol('x', 2), Matrix(Matrix::Zero(2, 2)));

  const Layout layout = Layout::From(values);

  LONGS_EQUAL(2 + 3 + 2, layout.totalDim);
  LONGS_EQUAL(0, layout.offsetOf(Symbol('x', 0)));
  LONGS_EQUAL(2, layout.offsetOf(Symbol('x', 1)));
  LONGS_EQUAL(5, layout.offsetOf(Symbol('x', 2)));
  LONGS_EQUAL(2, layout.rowDimOf(Symbol('x', 0)));
  LONGS_EQUAL(3, layout.rowDimOf(Symbol('x', 1)));
  LONGS_EQUAL(2, layout.rowDimOf(Symbol('x', 2)));
}

TEST(Layout, OffsetOfMissingKeyThrows) {
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 2)));
  const Layout layout = Layout::From(values);

  CHECK_EXCEPTION(layout.offsetOf(Symbol('x', 99)), std::out_of_range);
  CHECK_EXCEPTION(layout.rowDimOf(Symbol('x', 99)), std::out_of_range);
  CHECK_EXCEPTION(layout.sliceOf(Symbol('x', 99)), std::out_of_range);
  EXPECT(!layout.contains(Symbol('x', 99)));
  EXPECT(layout.contains(Symbol('x', 0)));
}

TEST(Layout, ConformsToMatchingValues) {
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 3)));
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(3, 3)));
  const Layout layout = Layout::From(values);

  EXPECT(layout.conformsTo(values));

  // A Values with the same keys but a different row dim should not conform.
  Values mismatched;
  mismatched.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 3)));
  mismatched.insert(Symbol('x', 1), Matrix(Matrix::Zero(2, 3)));  // wrong row dim
  EXPECT(!layout.conformsTo(mismatched));

  // A Values missing a key should not conform.
  Values dropped;
  dropped.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 3)));
  EXPECT(!layout.conformsTo(dropped));

  // A Values with an extra key should not conform.
  Values extra;
  extra.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 3)));
  extra.insert(Symbol('x', 1), Matrix(Matrix::Zero(3, 3)));
  extra.insert(Symbol('x', 2), Matrix(Matrix::Zero(2, 3)));
  EXPECT(!layout.conformsTo(extra));
}

TEST(Layout, StackUnstackRoundTrip) {
  // Heterogeneous row dims with a non-trivial column count.
  Values values;
  const Matrix A = (Matrix(2, 3) << 1, 2, 3, 4, 5, 6).finished();
  const Matrix B = (Matrix(3, 3) << 7, 8, 9, 10, 11, 12, 13, 14, 15).finished();
  const Matrix C = (Matrix(2, 3) << 16, 17, 18, 19, 20, 21).finished();
  values.insert(Symbol('x', 0), A);
  values.insert(Symbol('x', 1), B);
  values.insert(Symbol('x', 2), C);

  const Layout layout = Layout::From(values);
  const Matrix Ymat = layout.stack(values);
  LONGS_EQUAL(7, Ymat.rows());  // 2 + 3 + 2
  LONGS_EQUAL(3, Ymat.cols());
  EXPECT(assert_equal(A, Matrix(Ymat.block(0, 0, 2, 3)), 1e-15));
  EXPECT(assert_equal(B, Matrix(Ymat.block(2, 0, 3, 3)), 1e-15));
  EXPECT(assert_equal(C, Matrix(Ymat.block(5, 0, 2, 3)), 1e-15));

  // Round-trip: unstack the stacked matrix, get back the same per-key blocks.
  const Values vBack = layout.unstack(Ymat);
  EXPECT(assert_equal(A, vBack.at<Matrix>(Symbol('x', 0)), 1e-15));
  EXPECT(assert_equal(B, vBack.at<Matrix>(Symbol('x', 1)), 1e-15));
  EXPECT(assert_equal(C, vBack.at<Matrix>(Symbol('x', 2)), 1e-15));
}

TEST(Layout, StackInconsistentColsThrows) {
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 3)));
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(3, 4)));  // different col count
  const Layout layout = Layout::From(values);
  CHECK_EXCEPTION(layout.stack(values), std::invalid_argument);
}

TEST(Layout, UnstackWrongRowCountThrows) {
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 2)));
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(3, 2)));
  const Layout layout = Layout::From(values);

  // totalDim is 5; passing a matrix with rows != 5 should throw.
  CHECK_EXCEPTION(layout.unstack(Matrix::Zero(4, 2)), std::invalid_argument);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
