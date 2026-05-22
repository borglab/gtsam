/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRiemannianStaircase.cpp
 * @brief   Unit tests for RiemannianStaircaseOptimizer.
 *
 * Covers, in order:
 *  - assembleCertificate: ALM multipliers really yield KKT stationarity
 *    S·Y* ≈ 0 and a (near-)PSD certificate;
 *  - escapeSaddleAndLift: shape contract + O(α²) feasibility violation;
 *  - LOBPCG ↔ DenseEigen ↔ Spectra agree on verdict and λ_min;
 *  - End-to-end staircase on a 5-Rot2 ring (easy and wrong-winding init);
 *  - roundingSolution SVD truncation + sign-flip;
 *  - extractRotations<Rot2> on a hand-built RoundedSolution;
 *  - Layout API: offsets, totalDim, maxRowDim, conformsTo, stack/unstack.
 *
 * @author  Zhexin Xu
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/RiemannianStaircaseOptimizer.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
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

// Matrix-form (D=2) row-orthonormal fixture: each value is a 2x2 row-orthonormal
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
// Core certificate contract: the dual multipliers that ALM converges to,
// when fed into S = Q + Σ_m λ_m·A_m, must satisfy KKT stationarity
// S·Y* ≈ 0 *and* yield a PSD-up-to-η certificate. If either fails on a
// problem the staircase should be able to certify, the certificate (not
// ALM) is the bug. Run at K = 2, the base of the Rot2 ladder.
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
// Shape contract for the lift step: each variable gains exactly one column
// (= rank p+1), the first p columns are bit-identical to Y*, and the new
// column equals α·v restricted to that variable's row slice. Any drift in
// these invariants would silently corrupt the staircase climb.
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
// Lifting [Y* | α·v] from a feasible Y* gives h_m(Y_lift) = α²·v'A_m v —
// strictly quadratic in α. Doubling α must quadruple the violation norm.
// This justifies our descent step (the next ALM solve absorbs an O(α²)
// kick, but only because it really is O(α²) — not O(α)).
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
// Cross-validation between two verification backends. LOBPCG short-circuits
// via Cholesky when S+η·I is PSD, while DenseEigen always returns the true
// λ_min. Both must agree on the verdict and reach the same final rank. The
// most common cause of disagreement is a misaligned offset inside
// assembleCertificate — dense sees it directly, LOBPCG can miss it.
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
// Same cross-check, Spectra ↔ DenseEigen. Spectra has no Cholesky short-
// circuit, so we can additionally compare λ_min directly on certified runs
// (tighter check than the LOBPCG case can offer). The two verdicts plus
// λ_min must agree to Lanczos tolerance.
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
// End-to-end smoke on a tight 5-Rot2 ring: with a near-correct
// initialization, the SDP relaxation is tight at the starting rung and the
// certificate should pass after at most one lift. This is the "happy path"
// — if it fails, the outer loop is broken even before harder problems.
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
// Wrong-winding initialization: each pose is rotated *backwards*, so ALM at
// the starting rank will settle in a non-global minimum. The staircase
// must lift past it, regain a certifiable solution, and produce
// monotonically non-increasing costs across levels. This is the smallest
// example that genuinely exercises the saddle-escape mechanic.
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
// Layout determinism: Layout::From sorts keys before assigning offsets, so
// the layout is the same whatever order the Values were populated in. This
// matters because the assembled certificate's row/col ordering must be
// stable across staircase levels (lifts re-populate Values in different
// orders).
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
// Param sanity: pMin > pMax would silently run zero levels. validateParams
// must catch it at construction so the user fails fast instead of getting a
// "no progress" output.
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
// Direct Layout API coverage: offsetOf / rowDimOf / sliceOf / conformsTo /
// stack / unstack and Layout::From's handling of heterogeneous row dims.
// We test these in isolation (not just via the staircase) because they're
// the interface extension authors will reach for when adding a new variable
// type — they need a stable contract to code against.

using Layout = RiemannianStaircaseOptimizer::Layout;

// Heterogeneous row dims model the "Rot2 + Rot3 in one graph" extension
// case. The layout must still produce contiguous offsets and a totalDim
// equal to the row-dim sum — *any* future variable type plugs in here.
TEST(Layout, HeterogeneousRowDims) {
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

// All three accessors must throw on unknown keys instead of silently
// returning a default. Bad input here would otherwise corrupt the
// certificate's block placement.
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

// conformsTo is the pre-flight check before stack/unstack. It must reject:
// (a) values whose row dims disagree, (b) missing keys, (c) extra keys.
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

// stack/unstack must round-trip exactly (no accidental copy quirks): the
// staircase repeatedly stacks → reads/modifies → unstacks across lifts.
TEST(Layout, StackUnstackRoundTrip) {
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

// Inconsistent column counts across keys are an unrecoverable mistake — at
// rank p every block must be p columns wide. Fail loudly.
TEST(Layout, StackInconsistentColsThrows) {
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 3)));
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(3, 4)));
  const Layout layout = Layout::From(values);
  CHECK_EXCEPTION(layout.stack(values), std::invalid_argument);
}

// Unstack must row-check against totalDim; a wrong size means the caller is
// confusing layouts from different staircase levels.
TEST(Layout, UnstackWrongRowCountThrows) {
  Values values;
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 2)));
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(3, 2)));
  const Layout layout = Layout::From(values);

  CHECK_EXCEPTION(layout.unstack(Matrix::Zero(4, 2)), std::invalid_argument);
}

// maxRowDim drives the default rounding rank d in optimize() — rotation
// blocks dominate; row-vector blocks (translations / future scalar
// variables) must not pull the rank down to 1. Mix row dims explicitly.
TEST(Layout, MaxRowDim) {
  Values values;
  values.insert(Symbol('t', 0), Matrix(Matrix::Zero(1, 4)));  // row-vector
  values.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 4)));  // Rot2-shaped
  values.insert(Symbol('x', 1), Matrix(Matrix::Zero(3, 4)));  // Rot3-shaped
  const Layout layout = Layout::From(values);

  LONGS_EQUAL(3, layout.maxRowDim());
}

/* ************************************************************************* */
// Initial values supplied at the intrinsic dim (e.g. 2×2 for Rot2) must
// still work when pMin is set wider than that. optimize() is responsible
// for zero-padding initials up to pMin so callers don't need to track pMin
// in their (compile-time-K) InsertQcqpValue calls.
TEST(RiemannianStaircase, AutoPadsInitialValuesToPMin) {
  using namespace RingFixture;
  constexpr size_t N = 5;
  constexpr double delta = 2.0 * M_PI / static_cast<double>(N);

  const NonlinearFactorGraph graph = RingGraph(N, delta);
  // Initial values at K = intrinsicDim = 2 (the user's natural choice).
  const Values initial = RingQcqpValuesD2(N, delta, /*perturbation=*/0.01);

  RiemannianStaircaseParams params;
  params.pMin = 4;              // wider than the intrinsic dim
  params.pMax = 5;
  params.alpha = 1e-2;
  params.eta = 1e-3;
  params.almParams->maxIterations = 50;
  params.almParams->initialMuEq = 10.0;
  params.almParams->muEqIncreaseRate = 2.0;
  params.almParams->absoluteViolationTolerance = 1e-8;
  params.almParams->relativeViolationTolerance = 1e-8;

  QcqpProblem qcqp(graph, params.pMin);
  const auto result =
      RiemannianStaircaseOptimizer(qcqp, initial, params).optimize();

  // Just running to completion (no QpCost dim-mismatch throw) is the test.
  // Certification on this tight ring should also still pass.
  EXPECT(result.certified);
}

/* ************************************************************************* */
// Conversely, an over-wide initial (cols > pMin) is a caller error and must
// surface as a clear invalid_argument — silently truncating would corrupt
// the run.
TEST(RiemannianStaircase, RejectsInitialWiderThanPMin) {
  Values initial;
  // Build a 2×4 initial but ask for pMin = 3.
  initial.insert(Symbol('x', 0), Matrix(Matrix::Zero(2, 4)));
  initial.insert(Symbol('x', 1), Matrix(Matrix::Zero(2, 4)));

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
      Symbol('x', 0), Symbol('x', 1), Rot2::fromAngle(0.0));

  RiemannianStaircaseParams params;
  params.pMin = 3;
  params.pMax = 4;
  QcqpProblem qcqp(graph, params.pMin);
  CHECK_EXCEPTION(
      RiemannianStaircaseOptimizer(qcqp, initial, params).optimize(),
      std::invalid_argument);
}

/* ************************************************************************* */
// roundingSolution SVD-truncates the stacked BM matrix down to rank d. On
// an input that's *already* rank d, truncation must be a no-op up to a
// sign flip on the last column (SVD left singular vectors carry an
// arbitrary sign, which the sign-flip step normalizes).
TEST(RiemannianStaircase, RoundingSolutionPreservesRankD) {
  Values values;
  // Two Rot2-shaped blocks (2 rows each) at width 4: rank ≤ 2 in column
  // space, achieved by placing R^T in the first 2 cols and zeros in the
  // last 2.
  const Rot2 R0 = Rot2::fromAngle(0.3);
  const Rot2 R1 = Rot2::fromAngle(-0.8);
  Matrix B0 = Matrix::Zero(2, 4);
  B0.leftCols<2>() = R0.matrix().transpose();
  Matrix B1 = Matrix::Zero(2, 4);
  B1.leftCols<2>() = R1.matrix().transpose();
  values.insert(Symbol('x', 0), B0);
  values.insert(Symbol('x', 1), B1);
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);

  const auto rounded =
      RiemannianStaircaseOptimizer::roundingSolution(values, layout, /*d=*/2);

  LONGS_EQUAL(2, rounded.d);
  LONGS_EQUAL(4, rounded.Yd.rows());  // totalDim = 2 + 2
  LONGS_EQUAL(2, rounded.Yd.cols());
  // Each 2×2 block must round to a det ≥ 0 rotation (the sign-flip step
  // enforces det ≥ 0 for the majority of rotation-sized blocks).
  const double det0 = rounded.Yd.block(0, 0, 2, 2).determinant();
  const double det1 = rounded.Yd.block(2, 0, 2, 2).determinant();
  EXPECT(det0 > 0.0);
  EXPECT(det1 > 0.0);
}

// extractRotations<Rot2> reads each rowDim=2 slice as a Rot2 (orthogonal
// projection via ClosestTo). On a hand-built RoundedSolution containing the
// transposes of known rotations, we must recover those rotations exactly.
TEST(RiemannianStaircase, ExtractRotationsRot2) {
  Values values;
  const Rot2 R0 = Rot2::fromAngle(0.25);
  const Rot2 R1 = Rot2::fromAngle(-1.10);
  values.insert(Symbol('x', 0), Matrix(R0.matrix().transpose()));
  values.insert(Symbol('x', 1), Matrix(R1.matrix().transpose()));
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);

  RiemannianStaircaseOptimizer::RoundedSolution rs;
  rs.Yd = layout.stack(values);
  rs.d = 2;
  rs.flipped = false;

  const auto extracted =
      RiemannianStaircaseOptimizer::extractRotations<Rot2>(rs, layout);
  LONGS_EQUAL(2, extracted.size());

  // Reassemble into a Values for assert_equal — extraction order is
  // unordered_map-dependent, so look up by key.
  Values out;
  for (auto& [key, R] : extracted) out.insert(key, R);
  EXPECT(assert_equal(R0, out.at<Rot2>(Symbol('x', 0)), 1e-12));
  EXPECT(assert_equal(R1, out.at<Rot2>(Symbol('x', 1)), 1e-12));
}

// extractRotations<RotT> must skip slices whose rowDim doesn't match d —
// e.g., a row-vector block sitting alongside rotation blocks. Otherwise a
// future PGO-style problem would crash inside RotT::ClosestTo on a 1×N
// slice.
TEST(RiemannianStaircase, ExtractRotationsSkipsNonRotationSlices) {
  Values values;
  const Rot2 R0 = Rot2::fromAngle(0.4);
  values.insert(Symbol('x', 0), Matrix(R0.matrix().transpose()));    // 2×2
  values.insert(Symbol('t', 0),
                Matrix((Matrix(1, 2) << 0.7, -0.3).finished()));     // 1×2
  const auto layout = RiemannianStaircaseOptimizer::Layout::From(values);

  RiemannianStaircaseOptimizer::RoundedSolution rs;
  rs.Yd = layout.stack(values);
  rs.d = 2;
  rs.flipped = false;

  const auto extracted =
      RiemannianStaircaseOptimizer::extractRotations<Rot2>(rs, layout);
  LONGS_EQUAL(1, extracted.size());
  LONGS_EQUAL(static_cast<long>(Symbol('x', 0)), static_cast<long>(extracted.front().first));
  EXPECT(assert_equal(R0, extracted.front().second, 1e-12));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
