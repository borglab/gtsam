/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGaussianBayesTree.cpp
 * @date Jul 8, 2010
 * @author Kai Ni
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/linear/linearExceptions.h>

#include <future>
#include <iostream>
#include <limits>
#include <vector>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

using Pairs = std::vector<std::pair<Key, Matrix>>;

namespace {
  const Key x1=1, x2=2, x3=3, x4=4;
  const SharedDiagonal chainNoise = noiseModel::Isotropic::Sigma(1, 0.5);
  const GaussianFactorGraph chain = {
      std::make_shared<JacobianFactor>(x2, I_1x1, x1, I_1x1, Vector{{1.}},
                                       chainNoise),
      std::make_shared<JacobianFactor>(x2, I_1x1, x3, I_1x1, Vector{{1.}},
                                       chainNoise),
      std::make_shared<JacobianFactor>(x3, I_1x1, x4, I_1x1, Vector{{1.}},
                                       chainNoise),
      std::make_shared<JacobianFactor>(x4, I_1x1, Vector{{1.}}, chainNoise)};
  const Ordering chainOrdering{x2, x1, x3, x4};

  /* ************************************************************************* */
  // Helper functions for below
  GaussianBayesTreeClique::shared_ptr LeafClique(const GaussianConditional& conditional)
  {
    return std::make_shared<GaussianBayesTreeClique>(
      std::make_shared<GaussianConditional>(conditional));
  }

  typedef std::vector<GaussianBayesTreeClique::shared_ptr> Children;

  GaussianBayesTreeClique::shared_ptr MakeClique(
    const GaussianConditional& conditional, const Children& children)
  {
    auto clique = LeafClique(conditional);
    clique->children.assign(children.begin(), children.end());
    for(Children::const_iterator child = children.begin(); child != children.end(); ++child)
      (*child)->parent_ = clique;
    return clique;
  }
}

/* ************************************************************************* */
/**
 * x1 - x2 - x3 - x4
 * x3 x4
 *    x2 x1 : x3
 *
 * x2 x1 x3 x4  b
 *  1  1        1
 *  1     1     1
 *        1  1  1
 *           1  1
 *
 *  1  0  0  1
 */
TEST( GaussianBayesTree, eliminate )
{
  GaussianBayesTree bt = *chain.eliminateMultifrontal(chainOrdering);

  Scatter scatter(chain);
  EXPECT_LONGS_EQUAL(4, scatter.size());
  EXPECT_LONGS_EQUAL(1, scatter.at(0).key);
  EXPECT_LONGS_EQUAL(2, scatter.at(1).key);
  EXPECT_LONGS_EQUAL(3, scatter.at(2).key);
  EXPECT_LONGS_EQUAL(4, scatter.at(3).key);

  const Matrix two{{2.}};
  const Matrix one = I_1x1;

  const GaussianConditional gc1(
      std::map<Key, Matrix>{
          {x3, Matrix21{{2.}, {0.}}},
          {x4, Matrix21{{2.}, {2.}}},
      },
      2, Vector2(2., 2.));
  const GaussianConditional gc2(
      Pairs{
          {x2, Matrix21{{-2. * sqrt(2.)}, {0.}}},
          {x1, Matrix21{{-sqrt(2.)}, {-sqrt(2.)}}},
          {x3, Matrix21{{-sqrt(2.)}, {sqrt(2.)}}},
      },
      2, Vector{{-2. * sqrt(2.), 0.}});

  GaussianBayesTree bayesTree_expected;
  bayesTree_expected.insertRoot(MakeClique(gc1, Children{LeafClique(gc2)}));

  EXPECT(assert_equal(bayesTree_expected, bt));
}

/* ************************************************************************* */
TEST( GaussianBayesTree, optimizeMultiFrontal )
{
  VectorValues expected = {{x1, Vector{{0.}}},
                           {x2, Vector{{1.}}},
                           {x3, Vector{{0.}}},
                           {x4, Vector{{1.}}}};

  VectorValues actual = chain.eliminateMultifrontal(chainOrdering)->optimize();
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(GaussianBayesTree, complicatedMarginal) {
  // Create the conditionals to go in the BayesTree
  const GaussianConditional gc1(
      Pairs{{11, Matrix{{0.0971}, {0}, {0}}},
            {12, Matrix{{0.3171, 0.4387}, {0.9502, 0.3816}, {0, 0.7655}}}},
      2, Vector3(0.2638, 0.1455, 0.1361));
  const GaussianConditional gc2(
      Pairs{
          {9, Matrix{{0.7952}, {0}, {0}}},
          {10, Matrix{{0.4456, 0.7547}, {0.6463, 0.2760}, {0, 0.6797}}},
          {11, Matrix{{0.6551}, {0.1626}, {0.1190}}},
          {12, (Matrix{{0.4984, 0.5853}, {0.9597, 0.2238}, {0.3404, 0.7513}})}},
      2, Vector3(0.4314, 0.9106, 0.1818));
  const GaussianConditional gc3(
      Pairs{{7, Matrix{{0.2551}, {0}, {0}}},
            {8, Matrix{{0.8909, 0.1386}, {0.9593, 0.1493}, {0, 0.2575}}},
            {11, Matrix{{0.8407}, {0.2543}, {0.8143}}}},
      2, Vector3(0.3998, 0.2599, 0.8001));
  const GaussianConditional gc4(
      Pairs{{5, Matrix{{0.2435}, {0}, {0}}},
            {6, Matrix{{0.4733, 0.1966}, {0.3517, 0.2511}, {0.8308, 0.0}}},
            // NOTE the non-upper-triangular form
            // here since this test was written when we had column permutations
            // from LDL.  The code still works currently (does not enfore
            // upper-triangularity in this case) but this test will need to be
            // redone if this stops working in the future
            {7, Matrix{{0.5853}, {0.5497}, {0.9172}}},
            {8, Matrix{{0.2858, 0.3804}, {0.7572, 0.5678}, {0.7537, 0.0759}}}},
      2, Vector3(0.8173, 0.8687, 0.0844));
  const GaussianConditional gc5(
      Pairs{{3, Matrix{{0.0540}, {0}, {0}}},
            {4, Matrix{{0.9340, 0.4694}, {0.1299, 0.0119}, {0, 0.3371}}},
            {6, Matrix{{0.1622, 0.5285}, {0.7943, 0.1656}, {0.3112, 0.6020}}}},
      2, Vector3(0.9619, 0.0046, 0.7749));
  const GaussianConditional gc6(
      Pairs{{1, Matrix{{0.2630}, {0}, {0}}},
            {2, Matrix{{0.7482, 0.2290}, {0.4505, 0.9133}, {0, 0.1524}}},
            {5, Matrix{{0.8258}, {0.5383}, {0.9961}}}},
      2, Vector3(0.0782, 0.4427, 0.1067));

  // Create the bayes tree:
  GaussianBayesTree bt;
  bt.insertRoot(MakeClique(
      gc1, Children{LeafClique(gc2),
                    MakeClique(gc3, Children{MakeClique(
                                        gc4, Children{LeafClique(gc5),
                                                      LeafClique(gc6)})})}));

  // Marginal on 5
  Matrix expectedCov{{236.5166}};
  //GaussianConditional actualJacobianChol = *bt.marginalFactor(5, EliminateCholesky);
  GaussianConditional actualJacobianQR = *bt.marginalFactor(5, EliminateQR);
  //EXPECT(assert_equal(actualJacobianChol, actualJacobianQR)); // Check that Chol and QR obtained marginals are the same
  LONGS_EQUAL(1, (long)actualJacobianQR.rows());
  LONGS_EQUAL(1, (long)actualJacobianQR.size());
  LONGS_EQUAL(5, (long)actualJacobianQR.keys()[0]);
  Matrix actualA = actualJacobianQR.getA(actualJacobianQR.begin());
  Matrix actualCov = (actualA.transpose() * actualA).inverse();
  EXPECT(assert_equal(expectedCov, actualCov, 1e-1));

  // Marginal on 6
  expectedCov = Matrix{{1015.8, 2886.2}, {2886.2, 8471.2}};
  //actualJacobianChol = bt.marginalFactor(6, EliminateCholesky);
  actualJacobianQR = *bt.marginalFactor(6, EliminateQR);
  //EXPECT(assert_equal(actualJacobianChol, actualJacobianQR)); // Check that Chol and QR obtained marginals are the same
  LONGS_EQUAL(2, (long)actualJacobianQR.rows());
  LONGS_EQUAL(1, (long)actualJacobianQR.size());
  LONGS_EQUAL(6, (long)actualJacobianQR.keys()[0]);
  actualA = actualJacobianQR.getA(actualJacobianQR.begin());
  actualCov = (actualA.transpose() * actualA).inverse();
  EXPECT(assert_equal(expectedCov, actualCov, 1e1));
}

/* ************************************************************************* */
namespace {
double computeError(const GaussianBayesTree& gbt, const Vector10& values) {
  pair<Matrix, Vector> Rd = GaussianFactorGraph(gbt).jacobian();
  return 0.5 * (Rd.first * values - Rd.second).squaredNorm();
}
}

/* ************************************************************************* */
TEST(GaussianBayesTree, ComputeSteepestDescentPointBT) {
  const GaussianConditional gc1(  //
      Pairs{{2, Matrix{{31.0, 32.0},
                       {0.0, 34.0},
                       {0.0, 0.0},
                       {0.0, 0.0},
                       {0.0, 0.0},
                       {0.0, 0.0}}},
            {3, Matrix{{35.0, 36.0},
                       {37.0, 38.0},
                       {41.0, 42.0},
                       {0.0, 44.0},
                       {0.0, 0.0},
                       {0.0, 0.0}}},
            {4, Matrix{{0.0, 0.0},
                       {0.0, 0.0},
                       {45.0, 46.0},
                       {47.0, 48.0},
                       {51.0, 52.0},
                       {0.0, 54.0}}}},
      3, Vector{{29.0, 30.0, 39.0, 40.0, 49.0, 50.0}}),
      gc2(Pairs{{0, Matrix{{3.0, 4.0}, {0.0, 6.0}, {0.0, 0.0}, {0.0, 0.0}}},
                {1, Matrix{{0.0, 0.0}, {0.0, 0.0}, {17.0, 18.0}, {0.0, 20.0}}},
                {2, Matrix{{0.0, 0.0}, {0.0, 0.0}, {21.0, 22.0}, {23.0, 24.0}}},
                {3, Matrix{{7.0, 8.0}, {9.0, 10.0}, {0.0, 0.0}, {0.0, 0.0}}},
                {4,
                 Matrix{
                     {11.0, 12.0}, {13.0, 14.0}, {25.0, 26.0}, {27.0, 28.0}}}},
          2, Vector{{1.0, 2.0, 15.0, 16.0}});

  // Create an arbitrary Bayes Tree
  GaussianBayesTree bt;
  bt.insertRoot(MakeClique(gc1, Children{LeafClique(gc2)}));

  // Compute the Hessian numerically
  Matrix hessian = numericalHessian<Vector10>(
      std::bind(&computeError, bt, std::placeholders::_1), Vector10::Zero());

  // Compute the gradient numerically
  Vector gradient = numericalGradient<Vector10>(
      std::bind(&computeError, bt, std::placeholders::_1), Vector10::Zero());

  // Compute the gradient using dense matrices
  Matrix augmentedHessian = GaussianFactorGraph(bt).augmentedHessian();
  LONGS_EQUAL(11, (long)augmentedHessian.cols());
  Vector denseMatrixGradient = -augmentedHessian.col(10).segment(0,10);
  EXPECT(assert_equal(gradient, denseMatrixGradient, 1e-5));

  // Compute the steepest descent point
  double step = -gradient.squaredNorm() / (gradient.transpose() * hessian * gradient)(0);
  Vector expected = gradient * step;

  // Known steepest descent point from Bayes' net version
  VectorValues expectedFromBN{{0, Vector2(0.000129034, 0.000688183)},
                              {1, Vector2(0.0109679, 0.0253767)},
                              {2, Vector2(0.0680441, 0.114496)},
                              {3, Vector2(0.16125, 0.241294)},
                              {4, Vector2(0.300134, 0.423233)}};

  // Compute the steepest descent point with the dogleg function
  VectorValues actual = bt.optimizeGradientSearch();

  // Check that points agree
  KeyVector keys {0, 1, 2, 3, 4};
  EXPECT(assert_equal(expected, actual.vector(keys), 1e-5));
  EXPECT(assert_equal(expectedFromBN, actual, 1e-5));

  // Check that point causes a decrease in error
  double origError = GaussianFactorGraph(bt).error(VectorValues::Zero(actual));
  double newError = GaussianFactorGraph(bt).error(actual);
  EXPECT(newError < origError);
}

/* ************************************************************************* */
TEST(GaussianBayesTree, determinant_and_smallestEigenvalue) {

  // create small factor graph
  GaussianFactorGraph fg;
  Key x1 = 2, x2 = 0, l1 = 1;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  fg.emplace_shared<JacobianFactor>(x1, 10 * I_2x2, -1.0 * Vector::Ones(2), unit2);
  fg.emplace_shared<JacobianFactor>(x2, 10 * I_2x2, x1, -10 * I_2x2, Vector2(2.0, -1.0), unit2);
  fg.emplace_shared<JacobianFactor>(l1, 5 * I_2x2, x1, -5 * I_2x2, Vector2(0.0, 1.0), unit2);
  fg.emplace_shared<JacobianFactor>(x2, -5 * I_2x2, l1, 5 * I_2x2, Vector2(-1.0, 1.5), unit2);

  // create corresponding Bayes tree:
  std::shared_ptr<gtsam::GaussianBayesTree> bt = fg.eliminateMultifrontal();
  Matrix H = fg.hessian().first;

  // test determinant
  // NOTE: the hessian of the factor graph is H = R'R where R is the matrix encoded by the bayes tree,
  // for this reason we have to take the sqrt
  double expectedDeterminant = sqrt(H.determinant()); // determinant computed from full matrix
  double actualDeterminant = bt->determinant();
  EXPECT_DOUBLES_EQUAL(expectedDeterminant,actualDeterminant,expectedDeterminant*1e-6);// relative tolerance
}

/* ************************************************************************* */
/// Test to expose bug in GaussianBayesTree::logDeterminant.
TEST(GaussianBayesTree, LogDeterminant) {
  using symbol_shorthand::L;
  using symbol_shorthand::X;

  // Create a factor graph that will result in
  // a bayes tree with at least 2 nodes.
  GaussianFactorGraph fg;
  Key x1 = X(1), x2 = X(2), l1 = L(1);
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  fg.emplace_shared<JacobianFactor>(x1, 10 * I_2x2, -1.0 * Vector2::Ones(), unit2);
  fg.emplace_shared<JacobianFactor>(x2, 10 * I_2x2, x1, -10 * I_2x2,
                                    Vector2(2.0, -1.0), unit2);
  fg.emplace_shared<JacobianFactor>(l1, 5 * I_2x2, x1, -5 * I_2x2, Vector2(0.0, 1.0), unit2);
  fg.emplace_shared<JacobianFactor>(x2, -5 * I_2x2, l1, 5 * I_2x2, Vector2(-1.0, 1.5), unit2);
  fg.emplace_shared<JacobianFactor>(x3, 10 * I_2x2, x2, -10 * I_2x2,
                                    Vector2(2.0, -1.0), unit2);
  fg.emplace_shared<JacobianFactor>(x3, 10 * I_2x2, -1.0 * Vector2::Ones(), unit2);

  // create corresponding Bayes net and Bayes tree:
  std::shared_ptr<gtsam::GaussianBayesNet> bn = fg.eliminateSequential();
  std::shared_ptr<gtsam::GaussianBayesTree> bt = fg.eliminateMultifrontal();

  // Test logDeterminant
  EXPECT_DOUBLES_EQUAL(bn->logDeterminant(), bt->logDeterminant(), 1e-9);
}

/* ************************************************************************* */
namespace substitution_fixture {

using Clique = GaussianBayesTreeClique;

struct Forest {
  const Ordering ordering{10, 20, 30, 40, 50, 90};
  const VectorValues::Dims dimensions{{10, 2}, {20, 2}, {30, 1},
                                       {40, 2}, {50, 1}, {90, 1}};
  const Matrix raw{{2., .7, .2, .1, 0., 0., 0., .6, 0.},
                   {0., 3., -.3, .4, 0., 0., 0., -.1, 0.},
                   {0., 0., 1.4, -.2, 0., .5, -.7, .2, 0.},
                   {0., 0., 0., 1.7, 0., .2, .9, -.4, 0.},
                   {0., 0., 0., 0., 1.8, 0., 0., .8, 0.},
                   {0., 0., 0., 0., 0., 2.1, .5, -.2, 0.},
                   {0., 0., 0., 0., 0., 0., 2.3, .6, 0.},
                   {0., 0., 0., 0., 0., 0., 0., 1.2, 0.},
                   {0., 0., 0., 0., 0., 0., 0., 0., .9}};
  const Vector9 sigmas{2., 4., .6, 1.3, .8, 1.2, .7, 1.6, 3.};
  const Vector9 stored{1., -.3, .4, 2., -.2, .7, .5, -.9, 3.};
  GaussianBayesTree tree;

  Forest() {
    const auto root = add({40, 50}, 2, 5, 3);
    const auto child = add({20, 40, 50}, 1, 2, 2, root);
    add({10, 20, 50}, 1, 0, 2, child);
    add({30, 50}, 1, 4, 1, root);
    add({90}, 1, 8, 1);
  }

  Clique::shared_ptr add(const KeyVector& keys, size_t frontals, size_t row,
                        size_t rows, const Clique::shared_ptr& parent = {}) {
    Pairs terms;
    for (Key key : keys) {
      size_t column = 0;
      for (const auto& [candidate, dimension] : dimensions) {
        if (candidate == key) break;
        column += dimension;
      }
      terms.emplace_back(key, raw.block(row, column, rows, dimensions.at(key)));
    }
    auto conditional = std::make_shared<GaussianConditional>(
        terms, frontals, stored.segment(row, rows),
        noiseModel::Diagonal::Sigmas(sigmas.segment(row, rows)));
    auto clique = std::make_shared<Clique>(conditional);
    clique->problemSize_ = 100;  // Exercise task splitting in TBB builds.
    tree.addClique(clique, parent);
    return clique;
  }

  Matrix whitened() const { return sigmas.cwiseInverse().asDiagonal() * raw; }
};

// Each pass matches a dense oracle with coupled R, non-unit sigmas, multiple
// frontals, disconnected roots, and a separator reaching past the parent.
TEST(GaussianBayesTree, SubstitutionsWhitenedForest) {
  const Forest fixture;
  const Matrix matrix = fixture.whitened();
  const Vector9 seed{.3, -.6, .5, 1.2, -.8, .4, .9, -.1, 2.};
  const VectorValues rhs(seed, fixture.dimensions);
  const Vector expected = matrix.triangularView<Eigen::Upper>().solve(seed);
  const Vector expectedTranspose =
      matrix.transpose().triangularView<Eigen::Lower>().solve(seed);
  EXPECT(assert_equal(expected,
                      fixture.tree.backSubstitute(rhs).vector(fixture.ordering),
                      1e-12));
  EXPECT(assert_equal(expectedTranspose,
                      fixture.tree.backSubstituteTranspose(rhs).vector(fixture.ordering),
                      1e-12));
  const VectorValues adjoint = fixture.tree.backSubstitute(
      fixture.tree.backSubstituteTranspose(rhs));
  const Vector recovered = matrix.transpose() * matrix * adjoint.vector(fixture.ordering);
  EXPECT(assert_equal(Vector(seed), recovered, 1e-12));
}

// Substitution with the stored RHS agrees with optimize().
TEST(GaussianBayesTree, SubstitutionMatchesOptimize) {
  const Forest fixture;
  const Vector rhs = fixture.stored.cwiseQuotient(fixture.sigmas);
  EXPECT(assert_equal(fixture.tree.optimize(),
                      fixture.tree.backSubstitute(VectorValues(rhs, fixture.dimensions)),
                      1e-12));
  const Vector expected = fixture.raw.triangularView<Eigen::Upper>().solve(fixture.stored);
  EXPECT(assert_equal(expected, fixture.tree.optimize().vector(fixture.ordering), 1e-12));
}

// Omitted frontal blocks are zero, including an unsupervised disconnected root.
TEST(GaussianBayesTree, SubstitutionsSparseSeedsAndEmptyForest) {
  const Forest fixture;
  Vector seed = Vector::Zero(9);
  seed.head<2>() << .5, -.7;
  const VectorValues sparse{{10, Vector2{.5, -.7}}};
  const VectorValues complete(seed, fixture.dimensions);
  EXPECT(assert_equal(fixture.tree.backSubstitute(complete),
                      fixture.tree.backSubstitute(sparse), 1e-12));
  EXPECT(assert_equal(fixture.tree.backSubstituteTranspose(complete),
                      fixture.tree.backSubstituteTranspose(sparse), 1e-12));
  const GaussianBayesTree empty;
  EXPECT_LONGS_EQUAL(0, empty.backSubstitute({}).size());
  EXPECT_LONGS_EQUAL(0, empty.backSubstituteTranspose({}).size());
  CHECK_EXCEPTION(empty.backSubstitute(sparse), std::invalid_argument);
  CHECK_EXCEPTION(empty.backSubstituteTranspose(sparse), std::invalid_argument);
}

// Invalid seeds fail before parallel arithmetic starts.
TEST(GaussianBayesTree, SubstitutionsInvalidSeeds) {
  const Forest fixture;
  const std::vector<VectorValues> invalid{
      {{99, Vector1{1.}}}, {{10, Vector1{1.}}},
      {{10, Vector2{1., std::numeric_limits<double>::quiet_NaN()}}},
      {{50, Vector1{std::numeric_limits<double>::infinity()}}}};
  for (const auto& rhs : invalid) {
    CHECK_EXCEPTION(fixture.tree.backSubstitute(rhs), std::invalid_argument);
    CHECK_EXCEPTION(fixture.tree.backSubstituteTranspose(rhs), std::invalid_argument);
  }
}

// Singular conditionals and hard constraints have no finite inverse operator.
TEST(GaussianBayesTree, SubstitutionsSingularAndConstrained) {
  Forest fixture;
  fixture.tree.clique(10)->conditional()->getA()(0, 0) = 0.;
  CHECK_EXCEPTION(fixture.tree.backSubstitute({}), IndeterminateSystemException);
  CHECK_EXCEPTION(fixture.tree.backSubstituteTranspose({}), IndeterminateSystemException);
  GaussianBayesTree constrained;
  constrained.addClique(std::make_shared<Clique>(std::make_shared<GaussianConditional>(
      0, Vector1{2.}, Matrix11{{1.}}, noiseModel::Constrained::All(1))));
  CHECK_EXCEPTION(constrained.backSubstitute({}), std::invalid_argument);
  CHECK_EXCEPTION(constrained.backSubstituteTranspose({}), std::invalid_argument);
  EXPECT(assert_equal(Vector1{2.}, constrained.optimize().at(0), 1e-12));
}

// Tree, sequential, and MultifrontalSolver factorizations give matching solves.
TEST(GaussianBayesTree, SubstitutionsExistingFactorizations) {
  const VectorValues rhs{{x1, Vector1{.3}}, {x2, Vector1{-.5}},
                         {x3, Vector1{.7}}, {x4, Vector1{1.1}}};
  for (const GaussianFactorGraph::Eliminate& eliminate :
       {GaussianFactorGraph::Eliminate(EliminateQR),
        GaussianFactorGraph::Eliminate(EliminateCholesky)}) {
    const auto net = chain.eliminateSequential(chainOrdering, eliminate);
    const auto tree = chain.eliminateMultifrontal(chainOrdering, eliminate);
    // QR can choose different row signs for sequential/multifrontal cliques;
    // the composed information inverse is independent of those choices.
    EXPECT(assert_equal(net->backSubstitute(net->backSubstituteTranspose(rhs)),
                        tree->backSubstitute(tree->backSubstituteTranspose(rhs)),
                        1e-12));
  }
  MultifrontalSolver solver(chain, chainOrdering);
  solver.eliminateInPlace(chain);
  const auto tree = solver.computeBayesTree();
  const Vector actual = tree.backSubstitute(tree.backSubstituteTranspose(rhs)).vector();
  const Matrix information = chain.hessian().first;
  EXPECT(assert_equal(rhs.vector(), information * actual, 1e-12));
}

// Many siblings contribute to one separator without races or query mutation.
TEST(GaussianBayesTree, SubstitutionsParallelSiblingsAndConcurrentQueries) {
  constexpr Key rootKey = 1000;
  GaussianBayesTree tree;
  auto root = std::make_shared<Clique>(std::make_shared<GaussianConditional>(
      rootKey, Vector1{7.}, Matrix11{{2.}}));
  root->problemSize_ = 100;
  tree.addClique(root);
  VectorValues rhs, expected;
  double sum = 0.;
  for (Key key = 0; key < 64; ++key) {
    auto child = std::make_shared<Clique>(std::make_shared<GaussianConditional>(
        key, Vector1{3.}, Matrix11{{2.}}, rootKey, Matrix11{{.5}}));
    child->problemSize_ = 100;
    tree.addClique(child, root);
    rhs.insert(key, Vector1{.1 * (key + 1)});
    expected.insert(key, rhs.at(key) / 2.);
    sum += expected.at(key)[0];
  }
  expected.insert(rootKey, Vector1{-.25 * sum});
  VectorValues expectedAdjoint;
  const double rootAdjoint = expected.at(rootKey)[0] / 2.;
  expectedAdjoint.insert(rootKey, Vector1{rootAdjoint});
  for (Key key = 0; key < 64; ++key)
    expectedAdjoint.insert(key, Vector1{rhs.at(key)[0] / 4. - .25 * rootAdjoint});
  const auto query = [&tree, &rhs, &expected, &expectedAdjoint] {
    for (size_t repeat = 0; repeat < 8; ++repeat) {
      const auto intermediate = tree.backSubstituteTranspose(rhs);
      if (!intermediate.equals(expected, 1e-12) ||
          !tree.backSubstitute(intermediate).equals(expectedAdjoint, 1e-12))
        return false;
    }
    return true;
  };
  auto first = std::async(std::launch::async, query);
  auto second = std::async(std::launch::async, query);
  EXPECT(first.get());
  EXPECT(second.get());
  EXPECT_DOUBLES_EQUAL(7., root->conditional()->d()[0], 0.);
}

}  // namespace substitution_fixture
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
