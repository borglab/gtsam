/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGaussianJunctionTree.cpp
 * @date Jul 8, 2010
 * @author Kai Ni
 */

#include <iostream>
#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/base/debug.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/inference/BayesTree-inl.h>

using namespace std;
using namespace gtsam;

static const Index x2=0, x1=1, x3=2, x4=3;

static GaussianFactorGraph createChain() {

  typedef GaussianFactorGraph::sharedFactor Factor;
  SharedDiagonal model = noiseModel::Isotropic::Sigma(1, 0.5);
  Factor factor1(new JacobianFactor(x2, Matrix_(1,1,1.), x1, Matrix_(1,1,1.), Vector_(1,1.),  model));
  Factor factor2(new JacobianFactor(x2, Matrix_(1,1,1.), x3, Matrix_(1,1,1.), Vector_(1,1.),  model));
  Factor factor3(new JacobianFactor(x3, Matrix_(1,1,1.), x4, Matrix_(1,1,1.), Vector_(1,1.),  model));
  Factor factor4(new JacobianFactor(x4, Matrix_(1,1,1.), Vector_(1,1.),  model));

  GaussianFactorGraph fg;
  fg.push_back(factor1);
  fg.push_back(factor2);
  fg.push_back(factor3);
  fg.push_back(factor4);

  return fg;
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
TEST( GaussianJunctionTree, eliminate )
{
  GaussianFactorGraph fg = createChain();
  GaussianJunctionTree junctionTree(fg);
  BayesTree<GaussianConditional>::sharedClique rootClique = junctionTree.eliminate(&EliminateQR);

  typedef BayesTree<GaussianConditional>::sharedConditional sharedConditional;
  Matrix two = Matrix_(1,1,2.);
  Matrix one = Matrix_(1,1,1.);

  BayesTree<GaussianConditional> bayesTree_expected;
  Index keys_root[] = {x3,x4};
  Matrix rsd_root = Matrix_(2,3, 2., 2., 2., 0., 2., 2.);
  size_t dim_root[] = {1, 1, 1};
  sharedConditional root_expected(new GaussianConditional(keys_root, keys_root+2, 2,
      VerticalBlockView<Matrix>(rsd_root, dim_root, dim_root+3, 2), ones(2)));
  BayesTree<GaussianConditional>::sharedClique rootClique_expected(new BayesTree<GaussianConditional>::Clique(root_expected));

  Index keys_child[] = {x2,x1,x3};
  Matrix rsd_child = Matrix_(2,4, 2., 1., 1., 2., 0., -1., 1., 0.);
  size_t dim_child[] = {1, 1, 1, 1};
  sharedConditional child_expected(new GaussianConditional(keys_child, keys_child+3, 2,
      VerticalBlockView<Matrix>(rsd_child, dim_child, dim_child+4, 2), ones(2)));
  BayesTree<GaussianConditional>::sharedClique childClique_expected(new BayesTree<GaussianConditional>::Clique(child_expected));

  bayesTree_expected.insert(rootClique_expected);
  bayesTree_expected.insert(childClique_expected);

//  bayesTree_expected.insert(sharedConditional(new GaussianConditional(x4, Vector_(1,2.), two, Vector_(1,1.))));
//  bayesTree_expected.insert(sharedConditional(new GaussianConditional(x3, Vector_(1,2.), two, x4, two, Vector_(1,1.))));
//  bayesTree_expected.insert(sharedConditional(new GaussianConditional(x1, Vector_(1,0.), one*(-1), x3, one, Vector_(1,1.))));
//  bayesTree_expected.insert(sharedConditional(new GaussianConditional(x2, Vector_(1,2.), two, x1, one, x3, one, Vector_(1,1.))));
  CHECK(assert_equal(*bayesTree_expected.root(), *rootClique));
  EXPECT(assert_equal(*(bayesTree_expected.root()->children().front()), *(rootClique->children().front())));
}

/* ************************************************************************* */
TEST( GaussianJunctionTree, GBNConstructor )
{
  GaussianFactorGraph fg = createChain();
  GaussianJunctionTree jt(fg);
  BayesTree<GaussianConditional>::sharedClique root = jt.eliminate(&EliminateQR);
  BayesTree<GaussianConditional> expected;
  expected.insert(root);

  GaussianBayesNet bn(*GaussianSequentialSolver(fg).eliminate());
  BayesTree<GaussianConditional> actual(bn);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianJunctionTree, optimizeMultiFrontal )
{
  GaussianFactorGraph fg = createChain();
  GaussianJunctionTree tree(fg);

  VectorValues actual = tree.optimize(&EliminateQR);
  VectorValues expected(vector<size_t>(4,1));
  expected[x1] = Vector_(1, 0.);
  expected[x2] = Vector_(1, 1.);
  expected[x3] = Vector_(1, 0.);
  expected[x4] = Vector_(1, 1.);
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(GaussianJunctionTree, complicatedMarginal) {

  // Create the conditionals to go in the BayesTree
  GaussianConditional::shared_ptr R_1_2(new GaussianConditional(
      pair_list_of
          (1, (Matrix(3,1) <<
              0.2630,
              0,
              0).finished())
          (2, (Matrix(3,2) <<
              0.7482,    0.2290,
              0.4505,    0.9133,
                   0,    0.1524).finished())
          (5, (Matrix(3,1) <<
              0.8258,
              0.5383,
              0.9961).finished()),
      2, (Vector(3) << 0.0782, 0.4427, 0.1067).finished(), ones(3)));
  GaussianConditional::shared_ptr R_3_4(new GaussianConditional(
      pair_list_of
          (3, (Matrix(3,1) <<
              0.0540,
              0,
              0).finished())
          (4, (Matrix(3,2) <<
              0.9340,    0.4694,
              0.1299,    0.0119,
                   0,    0.3371).finished())
          (6, (Matrix(3,2) <<
              0.1622,    0.5285,
              0.7943,    0.1656,
              0.3112,    0.6020).finished()),
      2, (Vector(3) << 0.9619, 0.0046, 0.7749).finished(), ones(3)));
//  GaussianConditional::shared_ptr R_5_6(new GaussianConditional(
//      pair_list_of
//          (5, (Matrix(3,1) <<
//              0.2435,
//              0,
//              0).finished())
//          (6, (Matrix(3,2) <<
//              0.4733,    0.1966,
//              0.9022,    0.0979,
//                 0.0,    0.2312).finished())     // Attempted to recreate without permutation
//          (7, (Matrix(3,1) <<
//              0.5853,
//              1.0589,
//              0.1487).finished())
//          (8, (Matrix(3,2) <<
//              0.2858,    0.3804,
//              0.9893,    0.2912,
//              0.4035,    0.4933).finished()),
//      2, (Vector(3) << 0.8173, 0.4164, 0.7671).finished(), ones(3)));
  GaussianConditional::shared_ptr R_5_6(new GaussianConditional(
      pair_list_of
          (5, (Matrix(3,1) <<
              0.2435,
              0,
              0).finished())
          (6, (Matrix(3,2) <<
              0.4733,    0.1966,
              0.3517,    0.2511,
              0.8308,    0.0).finished()) // NOTE the non-upper-triangular form
              // here since this test was written when we had column permutations
              // from LDL.  The code still works currently (does not enfore
              // upper-triangularity in this case) but this test will need to be
              // redone if this stops working in the future
          (7, (Matrix(3,1) <<
              0.5853,
              0.5497,
              0.9172).finished())
          (8, (Matrix(3,2) <<
              0.2858,    0.3804,
              0.7572,    0.5678,
              0.7537,    0.0759).finished()),
      2, (Vector(3) << 0.8173, 0.8687, 0.0844).finished(), ones(3)));
  GaussianConditional::shared_ptr R_7_8(new GaussianConditional(
      pair_list_of
          (7, (Matrix(3,1) <<
              0.2551,
              0,
              0).finished())
          (8, (Matrix(3,2) <<
              0.8909,    0.1386,
              0.9593,    0.1493,
                   0,    0.2575).finished())
          (11, (Matrix(3,1) <<
              0.8407,
              0.2543,
              0.8143).finished()),
      2, (Vector(3) << 0.3998, 0.2599, 0.8001).finished(), ones(3)));
  GaussianConditional::shared_ptr R_9_10(new GaussianConditional(
      pair_list_of
          (9, (Matrix(3,1) <<
              0.7952,
              0,
              0).finished())
          (10, (Matrix(3,2) <<
              0.4456,    0.7547,
              0.6463,    0.2760,
                   0,    0.6797).finished())
          (11, (Matrix(3,1) <<
              0.6551,
              0.1626,
              0.1190).finished())
          (12, (Matrix(3,2) <<
              0.4984,    0.5853,
              0.9597,    0.2238,
              0.3404,    0.7513).finished()),
      2, (Vector(3) << 0.4314, 0.9106, 0.1818).finished(), ones(3)));
  GaussianConditional::shared_ptr R_11_12(new GaussianConditional(
      pair_list_of
          (11, (Matrix(3,1) <<
              0.0971,
              0,
              0).finished())
          (12, (Matrix(3,2) <<
              0.3171,    0.4387,
              0.9502,    0.3816,
                   0,    0.7655).finished()),
      2, (Vector(3) << 0.2638, 0.1455, 0.1361).finished(), ones(3)));

  // Gaussian Bayes Tree
  typedef BayesTree<GaussianConditional> GaussianBayesTree;
  typedef GaussianBayesTree::Clique Clique;
  typedef GaussianBayesTree::sharedClique sharedClique;

  // Create Bayes Tree
  GaussianBayesTree bt;
  bt.insert(sharedClique(new Clique(R_11_12)));
  bt.insert(sharedClique(new Clique(R_9_10)));
  bt.insert(sharedClique(new Clique(R_7_8)));
  bt.insert(sharedClique(new Clique(R_5_6)));
  bt.insert(sharedClique(new Clique(R_3_4)));
  bt.insert(sharedClique(new Clique(R_1_2)));

  // Marginal on 5
  Matrix expectedCov = (Matrix(1,1) << 236.5166).finished();
  JacobianFactor::shared_ptr actualJacobianChol= boost::dynamic_pointer_cast<JacobianFactor>(
      bt.marginalFactor(5, EliminateCholesky));
  JacobianFactor::shared_ptr actualJacobianQR = boost::dynamic_pointer_cast<JacobianFactor>(
      bt.marginalFactor(5, EliminateQR));
  CHECK(assert_equal(*actualJacobianChol, *actualJacobianQR)); // Check that Chol and QR obtained marginals are the same
  LONGS_EQUAL(1, actualJacobianChol->rows());
  LONGS_EQUAL(1, actualJacobianChol->size());
  LONGS_EQUAL(5, actualJacobianChol->keys()[0]);
  Matrix actualA = actualJacobianChol->getA(actualJacobianChol->begin());
  Matrix actualCov = inverse(actualA.transpose() * actualA);
  EXPECT(assert_equal(expectedCov, actualCov, 1e-1));

  // Marginal on 6
//  expectedCov = (Matrix(2,2) <<
//      8471.2, 2886.2,
//      2886.2, 1015.8).finished();
  expectedCov = (Matrix(2,2) <<
      1015.8,    2886.2,
      2886.2,    8471.2).finished();
  actualJacobianChol = boost::dynamic_pointer_cast<JacobianFactor>(
      bt.marginalFactor(6, EliminateCholesky));
  actualJacobianQR = boost::dynamic_pointer_cast<JacobianFactor>(
      bt.marginalFactor(6, EliminateQR));
  CHECK(assert_equal(*actualJacobianChol, *actualJacobianQR)); // Check that Chol and QR obtained marginals are the same
  LONGS_EQUAL(2, actualJacobianChol->rows());
  LONGS_EQUAL(1, actualJacobianChol->size());
  LONGS_EQUAL(6, actualJacobianChol->keys()[0]);
  actualA = actualJacobianChol->getA(actualJacobianChol->begin());
  actualCov = inverse(actualA.transpose() * actualA);
  EXPECT(assert_equal(expectedCov, actualCov, 1e1));

}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
