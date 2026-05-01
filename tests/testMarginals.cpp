/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testMarginals.cpp
 * @brief
 * @author Richard Roberts
 * @date May 14, 2012
 */

#include <CppUnitLite/TestHarness.h>

// for all nonlinear keys
#include <gtsam/inference/Symbol.h>

// for points and poses
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

// for modeling measurement uncertainty - all models included here
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/config.h>

// add in headers for specific factors
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

namespace {

/* ************************************************************************* */
KeyVector uniqueSortedKeys(const KeyVector& keys) {
  KeyVector result = keys;
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

/* ************************************************************************* */
GaussianFactorGraph legacyJointFactorGraph(
    const GaussianFactorGraph& graph, const Ordering& ordering,
    const KeyVector& variables, Marginals::Factorization factorization) {
  const KeyVector sortedVariables = uniqueSortedKeys(variables);
  if (factorization == Marginals::CHOLESKY) {
    GaussianBayesTree bayesTree =
        *graph.eliminateMultifrontal(ordering, EliminatePreferCholesky);
    if (sortedVariables.size() == 2) {
      return *bayesTree.joint(sortedVariables[0], sortedVariables[1],
                              EliminatePreferCholesky);
    }
    return GaussianFactorGraph(*graph.marginalMultifrontalBayesTree(
        sortedVariables, EliminatePreferCholesky));
  } else {
    GaussianBayesTree bayesTree =
        *graph.eliminateMultifrontal(ordering, EliminateQR);
    if (sortedVariables.size() == 2) {
      return *bayesTree.joint(sortedVariables[0], sortedVariables[1],
                              EliminateQR);
    }
    return GaussianFactorGraph(
        *graph.marginalMultifrontalBayesTree(sortedVariables, EliminateQR));
  }
}

/* ************************************************************************* */
Matrix legacyJointCovariance(const GaussianFactorGraph& graph,
                             const Ordering& ordering,
                             const KeyVector& variables,
                             Marginals::Factorization factorization) {
  const KeyVector sortedVariables = uniqueSortedKeys(variables);
  if (sortedVariables.size() == 1) {
    GaussianBayesTree bayesTree =
        factorization == Marginals::CHOLESKY
            ? *graph.eliminateMultifrontal(ordering, EliminatePreferCholesky)
            : *graph.eliminateMultifrontal(ordering, EliminateQR);
    return bayesTree.marginalFactor(sortedVariables.front())
        ->information()
        .inverse();
  }

  const GaussianFactorGraph jointFG =
      legacyJointFactorGraph(graph, ordering, sortedVariables, factorization);
  const Matrix augmentedInfo = jointFG.augmentedHessian();
  const Matrix information = augmentedInfo.topLeftCorner(
      augmentedInfo.rows() - 1, augmentedInfo.cols() - 1);
  return information.inverse();
}

}  // namespace

TEST(Marginals, planarSLAMmarginals) {

  // Taken from PlanarSLAMSelfContained_advanced

  // create keys for variables
  Symbol x1('x',1), x2('x',2), x3('x',3);
  Symbol l1('l',1), l2('l',2);

  // create graph container and add factors to it
  NonlinearFactorGraph graph;

  /* add prior  */
  // gaussian for prior
  SharedDiagonal priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
  graph.addPrior(x1, priorMean, priorNoise);  // add the factor to the graph

  /* add odometry */
  // general noisemodel for odometry
  SharedDiagonal odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  Pose2 odometry(2.0, 0.0, 0.0); // create a measurement for both factors (the same in this case)
  // create between factors to represent odometry
  graph.emplace_shared<BetweenFactor<Pose2>>(x1, x2, odometry, odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2>>(x2, x3, odometry, odometryNoise);

  /* add measurements */
  // general noisemodel for measurements
  SharedDiagonal measurementNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.2));

  // create the measurement values - indices are (pose id, landmark id)
  Rot2 bearing11 = Rot2::fromDegrees(45),
     bearing21 = Rot2::fromDegrees(90),
     bearing32 = Rot2::fromDegrees(90);
  double range11 = sqrt(4.0+4.0),
       range21 = 2.0,
       range32 = 2.0;

  // create bearing/range factors
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x1, l1, bearing11, range11, measurementNoise);
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x2, l1, bearing21, range21, measurementNoise);
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x3, l2, bearing32, range32, measurementNoise);

  // linearization point for marginals
  Values soln;
  soln.insert(x1, Pose2(0.0, 0.0, 0.0));
  soln.insert(x2, Pose2(2.0, 0.0, 0.0));
  soln.insert(x3, Pose2(4.0, 0.0, 0.0));
  soln.insert(l1, Point2(2.0, 2.0));
  soln.insert(l2, Point2(4.0, 2.0));
  VectorValues soln_lin;
  soln_lin.insert(x1, Vector3(0.0, 0.0, 0.0));
  soln_lin.insert(x2, Vector3(2.0, 0.0, 0.0));
  soln_lin.insert(x3, Vector3(4.0, 0.0, 0.0));
  soln_lin.insert(l1, Vector2(2.0, 2.0));
  soln_lin.insert(l2, Vector2(4.0, 2.0));

  Matrix expectedx1(3,3);
  expectedx1 <<
      0.09, -7.1942452e-18, -1.27897692e-17,
      -7.1942452e-18,         0.09, 1.27897692e-17,
      -1.27897692e-17,  1.27897692e-17,         0.01;
  Matrix expectedx2(3,3);
  expectedx2 <<
      0.120967742, -0.00129032258, 0.00451612903,
      -0.00129032258,  0.158387097, 0.0206451613,
      0.00451612903,  0.0206451613, 0.0177419355;
  Matrix expectedx3(3,3);
  expectedx3 <<
      0.160967742, 0.00774193548,  0.00451612903,
      0.00774193548,   0.351935484, 0.0561290323,
      0.00451612903,  0.0561290323, 0.0277419355;
  Matrix expectedl1(2,2);
  expectedl1 <<
      0.168709677, -0.0477419355,
      -0.0477419355,   0.163548387;
  Matrix expectedl2(2,2);
  expectedl2 <<
      0.293870968, -0.104516129,
    -0.104516129,  0.391935484;

  auto testMarginals = [&] (Marginals marginals) {
    EXPECT(assert_equal(expectedx1, marginals.marginalCovariance(x1), 1e-8));
    EXPECT(assert_equal(expectedx2, marginals.marginalCovariance(x2), 1e-8));
    EXPECT(assert_equal(expectedx3, marginals.marginalCovariance(x3), 1e-8));
    EXPECT(assert_equal(expectedl1, marginals.marginalCovariance(l1), 1e-8));
    EXPECT(assert_equal(expectedl2, marginals.marginalCovariance(l2), 1e-8));
  };

  auto testJointMarginals = [&] (Marginals marginals) {
    // Check joint marginals for 3 variables
    Matrix expected_l2x1x3(8,8);
    expected_l2x1x3 <<
        0.293871159514111,  -0.104516127560770,   0.090000180000270,  -0.000000000000000,  -0.020000000000000,   0.151935669757191,  -0.104516127560770,  -0.050967744878460,
      -0.104516127560770,   0.391935664055174,   0.000000000000000,   0.090000180000270,   0.040000000000000,   0.007741936219615,   0.351935664055174,   0.056129031890193,
        0.090000180000270,   0.000000000000000,   0.090000180000270,  -0.000000000000000,   0.000000000000000,   0.090000180000270,   0.000000000000000,   0.000000000000000,
      -0.000000000000000,   0.090000180000270,  -0.000000000000000,   0.090000180000270,   0.000000000000000,  -0.000000000000000,   0.090000180000270,   0.000000000000000,
      -0.020000000000000,   0.040000000000000,   0.000000000000000,   0.000000000000000,   0.010000000000000,   0.000000000000000,   0.040000000000000,   0.010000000000000,
        0.151935669757191,   0.007741936219615,   0.090000180000270,  -0.000000000000000,   0.000000000000000,   0.160967924878730,   0.007741936219615,   0.004516127560770,
      -0.104516127560770,   0.351935664055174,   0.000000000000000,   0.090000180000270,   0.040000000000000,   0.007741936219615,   0.351935664055174,   0.056129031890193,
      -0.050967744878460,   0.056129031890193,   0.000000000000000,   0.000000000000000,   0.010000000000000,   0.004516127560770,   0.056129031890193,   0.027741936219615;
    KeyVector variables {x1, l2, x3};
    JointMarginal joint_l2x1x3 = marginals.jointMarginalCovariance(variables);
    EXPECT(assert_equal(Matrix(expected_l2x1x3.block(0,0,2,2)), Matrix(joint_l2x1x3(l2,l2)), 1e-6));
    EXPECT(assert_equal(Matrix(expected_l2x1x3.block(2,0,3,2)), Matrix(joint_l2x1x3(x1,l2)), 1e-6));
    EXPECT(assert_equal(Matrix(expected_l2x1x3.block(5,0,3,2)), Matrix(joint_l2x1x3(x3,l2)), 1e-6));

    EXPECT(assert_equal(Matrix(expected_l2x1x3.block(0,2,2,3)), Matrix(joint_l2x1x3(l2,x1)), 1e-6));
    EXPECT(assert_equal(Matrix(expected_l2x1x3.block(2,2,3,3)), Matrix(joint_l2x1x3(x1,x1)), 1e-6));
    EXPECT(assert_equal(Matrix(expected_l2x1x3.block(5,2,3,3)), Matrix(joint_l2x1x3(x3,x1)), 1e-6));

    EXPECT(assert_equal(Matrix(expected_l2x1x3.block(0,5,2,3)), Matrix(joint_l2x1x3(l2,x3)), 1e-6));
    EXPECT(assert_equal(Matrix(expected_l2x1x3.block(2,5,3,3)), Matrix(joint_l2x1x3(x1,x3)), 1e-6));
    EXPECT(assert_equal(Matrix(expected_l2x1x3.block(5,5,3,3)), Matrix(joint_l2x1x3(x3,x3)), 1e-6));

    // Check joint marginals for 2 variables (different code path than >2 variable case above)
    Matrix expected_l2x1(5,5);
    expected_l2x1 <<
        0.293871159514111,  -0.104516127560770,   0.090000180000270,  -0.000000000000000,  -0.020000000000000,
      -0.104516127560770,   0.391935664055174,   0.000000000000000,   0.090000180000270,   0.040000000000000,
        0.090000180000270,   0.000000000000000,   0.090000180000270,  -0.000000000000000,   0.000000000000000,
      -0.000000000000000,   0.090000180000270,  -0.000000000000000,   0.090000180000270,   0.000000000000000,
      -0.020000000000000,   0.040000000000000,   0.000000000000000,   0.000000000000000,   0.010000000000000;
    variables.resize(2);
    variables[0] = l2;
    variables[1] = x1;
    JointMarginal joint_l2x1 = marginals.jointMarginalCovariance(variables);
    EXPECT(assert_equal(Matrix(expected_l2x1.block(0,0,2,2)), Matrix(joint_l2x1(l2,l2)), 1e-6));
    EXPECT(assert_equal(Matrix(expected_l2x1.block(2,0,3,2)), Matrix(joint_l2x1(x1,l2)), 1e-6));
    EXPECT(assert_equal(Matrix(expected_l2x1.block(0,2,2,3)), Matrix(joint_l2x1(l2,x1)), 1e-6));
    EXPECT(assert_equal(Matrix(expected_l2x1.block(2,2,3,3)), Matrix(joint_l2x1(x1,x1)), 1e-6));

    // Check joint marginal for 1 variable (different code path than >1 variable cases above)
    variables.resize(1);
    variables[0] = x1;
    JointMarginal joint_x1 = marginals.jointMarginalCovariance(variables);
    EXPECT(assert_equal(expectedx1, Matrix(joint_l2x1(x1,x1)), 1e-6));
  };

  Marginals marginals;

  marginals = Marginals(graph, soln, Marginals::CHOLESKY);
  testMarginals(marginals);
  marginals = Marginals(graph, soln, Marginals::QR);
  testMarginals(marginals);
  testJointMarginals(marginals);

  GaussianFactorGraph gfg = *graph.linearize(soln);
  marginals = Marginals(gfg, soln_lin, Marginals::CHOLESKY);
  testMarginals(marginals);
  marginals = Marginals(gfg, soln_lin, Marginals::QR);
  testMarginals(marginals);
  testJointMarginals(marginals);

  const Ordering ordering = Ordering::Colamd(gfg);
  for (const auto factorization : {Marginals::CHOLESKY, Marginals::QR}) {
    Marginals orderedMarginals(gfg, soln_lin, ordering, factorization);
    const KeyVector unorderedVariables{x3, l2, x1};
    const Matrix actual =
        orderedMarginals.jointMarginalCovariance(unorderedVariables)
            .fullMatrix();
    const Matrix expected =
        legacyJointCovariance(gfg, ordering, unorderedVariables, factorization);
    EXPECT(assert_equal(expected, actual, 1e-8));
  }
}

/* ************************************************************************* */
TEST(Marginals, order) {
  NonlinearFactorGraph fg;
  fg.addPrior(0, Pose2(), noiseModel::Unit::Create(3));
  fg.emplace_shared<BetweenFactor<Pose2>>(0, 1, Pose2(1,0,0), noiseModel::Unit::Create(3));
  fg.emplace_shared<BetweenFactor<Pose2>>(1, 2, Pose2(1,0,0), noiseModel::Unit::Create(3));
  fg.emplace_shared<BetweenFactor<Pose2>>(2, 3, Pose2(1,0,0), noiseModel::Unit::Create(3));

  Values vals;
  vals.insert(0, Pose2());
  vals.insert(1, Pose2(1,0,0));
  vals.insert(2, Pose2(2,0,0));
  vals.insert(3, Pose2(3,0,0));

  vals.insert(100, Point2(0,1));
  vals.insert(101, Point2(1,1));

  fg.emplace_shared<BearingRangeFactor<Pose2,Point2>>(0, 100,
    vals.at<Pose2>(0).bearing(vals.at<Point2>(100)),
    vals.at<Pose2>(0).range(vals.at<Point2>(100)), noiseModel::Unit::Create(2));
  fg.emplace_shared<BearingRangeFactor<Pose2,Point2>>(0, 101,
    vals.at<Pose2>(0).bearing(vals.at<Point2>(101)),
    vals.at<Pose2>(0).range(vals.at<Point2>(101)), noiseModel::Unit::Create(2));

  fg.emplace_shared<BearingRangeFactor<Pose2,Point2>>(1, 100,
    vals.at<Pose2>(1).bearing(vals.at<Point2>(100)),
    vals.at<Pose2>(1).range(vals.at<Point2>(100)), noiseModel::Unit::Create(2));
  fg.emplace_shared<BearingRangeFactor<Pose2,Point2>>(1, 101,
    vals.at<Pose2>(1).bearing(vals.at<Point2>(101)),
    vals.at<Pose2>(1).range(vals.at<Point2>(101)), noiseModel::Unit::Create(2));

  fg.emplace_shared<BearingRangeFactor<Pose2,Point2>>(2, 100,
    vals.at<Pose2>(2).bearing(vals.at<Point2>(100)),
    vals.at<Pose2>(2).range(vals.at<Point2>(100)), noiseModel::Unit::Create(2));
  fg.emplace_shared<BearingRangeFactor<Pose2,Point2>>(2, 101,
    vals.at<Pose2>(2).bearing(vals.at<Point2>(101)),
    vals.at<Pose2>(2).range(vals.at<Point2>(101)), noiseModel::Unit::Create(2));

  fg.emplace_shared<BearingRangeFactor<Pose2,Point2>>(3, 100,
    vals.at<Pose2>(3).bearing(vals.at<Point2>(100)),
    vals.at<Pose2>(3).range(vals.at<Point2>(100)), noiseModel::Unit::Create(2));
  fg.emplace_shared<BearingRangeFactor<Pose2,Point2>>(3, 101,
    vals.at<Pose2>(3).bearing(vals.at<Point2>(101)),
    vals.at<Pose2>(3).range(vals.at<Point2>(101)), noiseModel::Unit::Create(2));

  auto testMarginals = [&] (Marginals marginals, KeySet set) {
    KeyVector keys(set.begin(), set.end());
    JointMarginal joint = marginals.jointMarginalCovariance(keys);

    LONGS_EQUAL(3, (long)joint(0,0).rows());
    LONGS_EQUAL(3, (long)joint(1,1).rows());
    LONGS_EQUAL(3, (long)joint(2,2).rows());
    LONGS_EQUAL(3, (long)joint(3,3).rows());
    LONGS_EQUAL(2, (long)joint(100,100).rows());
    LONGS_EQUAL(2, (long)joint(101,101).rows());
  };

  Marginals marginals(fg, vals);
  KeySet set = fg.keys();
  testMarginals(marginals, set);

  GaussianFactorGraph gfg = *fg.linearize(vals);
  marginals = Marginals(gfg, vals);
  set = gfg.keys();
  testMarginals(marginals, set);
}

#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
/* ************************************************************************* */
TEST(Marginals, orderingEquivalence) {
  NonlinearFactorGraph graph;
  graph.addPrior(0, Pose2(), noiseModel::Unit::Create(3));
  graph.emplace_shared<BetweenFactor<Pose2>>(0, 1, Pose2(1, 0, 0),
                                             noiseModel::Unit::Create(3));
  graph.emplace_shared<BetweenFactor<Pose2>>(1, 2, Pose2(1, 0, 0),
                                             noiseModel::Unit::Create(3));
  graph.emplace_shared<BetweenFactor<Pose2>>(2, 3, Pose2(1, 0, 0),
                                             noiseModel::Unit::Create(3));
  graph.emplace_shared<BetweenFactor<Pose2>>(3, 4, Pose2(1, 0, 0),
                                             noiseModel::Unit::Create(3));

  Values values;
  values.insert(0, Pose2());
  values.insert(1, Pose2(1, 0, 0));
  values.insert(2, Pose2(2, 0, 0));
  values.insert(3, Pose2(3, 0, 0));
  values.insert(4, Pose2(4, 0, 0));

  GaussianFactorGraph linearGraph = *graph.linearize(values);
  const Ordering colamd = Ordering::Create(Ordering::COLAMD, linearGraph);
  const Ordering metis = Ordering::Create(Ordering::METIS, linearGraph);
  const KeyVector query{4, 1, 3};

  for (const auto factorization : {Marginals::CHOLESKY, Marginals::QR}) {
    Marginals colamdMarginals(linearGraph, values, colamd, factorization);
    Marginals metisMarginals(linearGraph, values, metis, factorization);
    EXPECT(assert_equal(
        colamdMarginals.jointMarginalCovariance(query).fullMatrix(),
        metisMarginals.jointMarginalCovariance(query).fullMatrix(), 1e-8));
  }
}
#endif

/* ************************************************************************* */
TEST(Marginals, jointCovarianceMatchesFullHessianInverse) {
  auto dimsForKeys = [](const Values& values, const KeyVector& keys) {
    std::vector<size_t> dims;
    dims.reserve(keys.size());
    for (Key key : keys) {
      dims.push_back(values.at(key).dim());
    }
    return dims;
  };

  auto blockOffsets = [](const std::vector<size_t>& dims) {
    std::vector<size_t> offsets(dims.size() + 1, 0);
    for (size_t index = 0; index < dims.size(); ++index) {
      offsets[index + 1] = offsets[index] + dims[index];
    }
    return offsets;
  };

  auto covarianceBlockFromFullHessian = [&](const GaussianFactorGraph& graph,
                                            const Values& values,
                                            const KeyVector& queryKeys) {
    const KeyVector orderedQuery = uniqueSortedKeys(queryKeys);
    const KeySet graphKeys = graph.keys();
    KeyVector allKeys(graphKeys.begin(), graphKeys.end());
    const Ordering ordering(allKeys);
    const auto [information, eta] = graph.hessian(ordering);
    (void)eta;
    const Matrix covariance = information.inverse();

    const std::vector<size_t> allDims = dimsForKeys(values, allKeys);
    const std::vector<size_t> allOffsets = blockOffsets(allDims);
    const std::vector<size_t> queryDims = dimsForKeys(values, orderedQuery);
    const std::vector<size_t> queryOffsets = blockOffsets(queryDims);

    FastMap<Key, size_t> indexByKey;
    for (size_t index = 0; index < allKeys.size(); ++index) {
      indexByKey[allKeys[index]] = index;
    }

    Matrix result = Matrix::Zero(queryOffsets.back(), queryOffsets.back());
    for (size_t rowIndex = 0; rowIndex < orderedQuery.size(); ++rowIndex) {
      const size_t sourceRow = indexByKey.at(orderedQuery[rowIndex]);
      const size_t rowDim = queryDims[rowIndex];
      for (size_t colIndex = 0; colIndex < orderedQuery.size(); ++colIndex) {
        const size_t sourceCol = indexByKey.at(orderedQuery[colIndex]);
        const size_t colDim = queryDims[colIndex];
        result.block(queryOffsets[rowIndex], queryOffsets[colIndex], rowDim,
                     colDim) =
            covariance.block(allOffsets[sourceRow], allOffsets[sourceCol],
                             rowDim, colDim);
      }
    }
    return result;
  };

  NonlinearFactorGraph graph;
  const auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  const auto odometryNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));

  graph.addPrior(0, Pose2(), priorNoise);
  graph.emplace_shared<BetweenFactor<Pose2>>(0, 1, Pose2(1, 0, 0),
                                             odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2>>(1, 2, Pose2(1, 0, 0),
                                             odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2>>(2, 3, Pose2(1, 0, 0),
                                             odometryNoise);

  Values values;
  values.insert(0, Pose2());
  values.insert(1, Pose2(1, 0, 0));
  values.insert(2, Pose2(2, 0, 0));
  values.insert(3, Pose2(3, 0, 0));

  const GaussianFactorGraph linearGraph = *graph.linearize(values);
  const KeyVector query{3, 1, 2};
  const Matrix expected =
      covarianceBlockFromFullHessian(linearGraph, values, query);

  for (const auto factorization : {Marginals::CHOLESKY, Marginals::QR}) {
    const Marginals marginals(linearGraph, values, factorization);
    const Matrix actual = marginals.jointMarginalCovariance(query).fullMatrix();
    EXPECT(assert_equal(expected, actual, 1e-8));
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
