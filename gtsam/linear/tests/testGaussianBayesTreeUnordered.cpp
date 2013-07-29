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

#include <iostream>
#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/base/debug.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/linear/GaussianJunctionTreeUnordered.h>
#include <gtsam/linear/GaussianBayesTreeUnordered.h>
#include <gtsam/linear/GaussianConditionalUnordered.h>

using namespace std;
using namespace gtsam;

#define TEST TEST_UNSAFE

namespace {
  const Key x1=1, x2=2, x3=3, x4=4;
  const SharedDiagonal chainNoise = noiseModel::Isotropic::Sigma(1, 0.5);
  const GaussianFactorGraphUnordered chain = list_of
    (JacobianFactorUnordered(x2, Matrix_(1,1,1.), x1, Matrix_(1,1,1.), Vector_(1,1.),  chainNoise))
    (JacobianFactorUnordered(x2, Matrix_(1,1,1.), x3, Matrix_(1,1,1.), Vector_(1,1.),  chainNoise))
    (JacobianFactorUnordered(x3, Matrix_(1,1,1.), x4, Matrix_(1,1,1.), Vector_(1,1.),  chainNoise))
    (JacobianFactorUnordered(x4, Matrix_(1,1,1.), Vector_(1,1.),  chainNoise));
  const OrderingUnordered chainOrdering = OrderingUnordered(list_of(x2)(x1)(x3)(x4));

  /* ************************************************************************* */
  // Helper functions for below
  GaussianBayesTreeCliqueUnordered::shared_ptr MakeClique(const GaussianConditionalUnordered& conditional)
  {
    return boost::make_shared<GaussianBayesTreeCliqueUnordered>(
      boost::make_shared<GaussianConditionalUnordered>(conditional));
  }

  template<typename CHILDREN>
  GaussianBayesTreeCliqueUnordered::shared_ptr MakeClique(
    const GaussianConditionalUnordered& conditional, const CHILDREN& children)
  {
    GaussianBayesTreeCliqueUnordered::shared_ptr clique =
      boost::make_shared<GaussianBayesTreeCliqueUnordered>(
      boost::make_shared<GaussianConditionalUnordered>(conditional));
    clique->children = children;
    BOOST_FOREACH(const GaussianBayesTreeCliqueUnordered::shared_ptr& child, children)
      child->parent_ = clique;
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
  GaussianBayesTreeUnordered bt = *chain.eliminateMultifrontal(chainOrdering);

  Matrix two = Matrix_(1,1,2.);
  Matrix one = Matrix_(1,1,1.);

  GaussianBayesTreeUnordered bayesTree_expected;
  bayesTree_expected.insertRoot(
    MakeClique(GaussianConditionalUnordered(pair_list_of (x3, Matrix_(2,1, 2., 0.)) (x4, Matrix_(2,1, 2., 2.)), 2, Vector_(2, 2., 2.)), list_of
      (MakeClique(GaussianConditionalUnordered(pair_list_of (x2, Matrix_(2,1, -2.*sqrt(2.), 0.)) (x1, Matrix_(2,1, -sqrt(2.), -sqrt(2.))) (x3, Matrix_(2,1, -sqrt(2.), sqrt(2.))), 2, Vector_(2, -2.*sqrt(2.), 0.))))));

  EXPECT(assert_equal(bayesTree_expected, bt));
}

/* ************************************************************************* */
TEST( GaussianBayesTree, optimizeMultiFrontal )
{
  VectorValuesUnordered expected = pair_list_of
    (x1, Vector_(1, 0.))
    (x2, Vector_(1, 1.))
    (x3, Vector_(1, 0.))
    (x4, Vector_(1, 1.));

  VectorValuesUnordered actual = chain.eliminateMultifrontal(chainOrdering)->optimize();
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(GaussianBayesTree, complicatedMarginal) {

  // Create the conditionals to go in the BayesTree
  GaussianBayesTreeUnordered bt;
  bt.insertRoot(
    MakeClique(GaussianConditionalUnordered(pair_list_of (11, (Matrix(3,1) << 0.0971, 0, 0).finished())
                                                         (12, (Matrix(3,2) << 0.3171, 0.4387,  0.9502, 0.3816,  0, 0.7655).finished()),
                                            2, (Vector(3) << 0.2638, 0.1455, 0.1361).finished()), list_of
      (MakeClique(GaussianConditionalUnordered(pair_list_of (9, (Matrix(3,1) << 0.7952, 0, 0).finished())
                                                            (10, (Matrix(3,2) << 0.4456, 0.7547, 0.6463, 0.2760, 0, 0.6797).finished())
                                                            (11, (Matrix(3,1) << 0.6551, 0.1626, 0.1190).finished())
                                                            (12, (Matrix(3,2) << 0.4984, 0.5853, 0.9597, 0.2238, 0.3404, 0.7513).finished()),
                                               2, (Vector(3) << 0.4314, 0.9106, 0.1818).finished())))
      (MakeClique(GaussianConditionalUnordered(pair_list_of (7, (Matrix(3,1) << 0.2551, 0, 0).finished())
                                                            (8, (Matrix(3,2) << 0.8909, 0.1386, 0.9593, 0.1493, 0, 0.2575).finished())
                                                            (11, (Matrix(3,1) << 0.8407, 0.2543, 0.8143).finished()),
                                               2, (Vector(3) << 0.3998, 0.2599, 0.8001).finished()), list_of
          (MakeClique(GaussianConditionalUnordered(pair_list_of (5, (Matrix(3,1) << 0.2435, 0, 0).finished())
                                                                (6, (Matrix(3,2) << 0.4733, 0.1966, 0.3517, 0.2511, 0.8308,    0.0).finished())
                                                                // NOTE the non-upper-triangular form
                                                                // here since this test was written when we had column permutations
                                                                // from LDL.  The code still works currently (does not enfore
                                                                // upper-triangularity in this case) but this test will need to be
                                                                // redone if this stops working in the future
                                                                (7, (Matrix(3,1) << 0.5853, 0.5497, 0.9172).finished())
                                                                (8, (Matrix(3,2) << 0.2858, 0.3804, 0.7572, 0.5678, 0.7537, 0.0759).finished()),
                                                  2, (Vector(3) << 0.8173, 0.8687, 0.0844).finished()), list_of
              (MakeClique(GaussianConditionalUnordered(pair_list_of (3, (Matrix(3,1) << 0.0540, 0, 0).finished())
                                                                    (4, (Matrix(3,2) << 0.9340, 0.4694, 0.1299, 0.0119, 0, 0.3371).finished())
                                                                    (6, (Matrix(3,2) << 0.1622, 0.5285, 0.7943, 0.1656, 0.3112, 0.6020).finished()),
                                                      2, (Vector(3) << 0.9619, 0.0046, 0.7749).finished())))
              (MakeClique(GaussianConditionalUnordered(pair_list_of (1, (Matrix(3,1) << 0.2630, 0, 0).finished())
                                                                    (2, (Matrix(3,2) << 0.7482, 0.2290, 0.4505, 0.9133, 0, 0.1524).finished())
                                                                    (5, (Matrix(3,1) << 0.8258, 0.5383, 0.9961).finished()),
                                                      2, (Vector(3) << 0.0782, 0.4427, 0.1067).finished())))))))));

  // Marginal on 5
  Matrix expectedCov = (Matrix(1,1) << 236.5166).finished();
  //GaussianConditionalUnordered actualJacobianChol = *bt.marginalFactor(5, EliminateCholesky);
  GaussianConditionalUnordered actualJacobianQR = *bt.marginalFactor(5, EliminateQRUnordered);
  //EXPECT(assert_equal(actualJacobianChol, actualJacobianQR)); // Check that Chol and QR obtained marginals are the same
  LONGS_EQUAL(1, (long)actualJacobianQR.rows());
  LONGS_EQUAL(1, (long)actualJacobianQR.size());
  LONGS_EQUAL(5, (long)actualJacobianQR.keys()[0]);
  Matrix actualA = actualJacobianQR.getA(actualJacobianQR.begin());
  Matrix actualCov = inverse(actualA.transpose() * actualA);
  EXPECT(assert_equal(expectedCov, actualCov, 1e-1));

  // Marginal on 6
//  expectedCov = (Matrix(2,2) <<
//      8471.2, 2886.2,
//      2886.2, 1015.8).finished();
  expectedCov = (Matrix(2,2) <<
      1015.8,    2886.2,
      2886.2,    8471.2).finished();
  //actualJacobianChol = bt.marginalFactor(6, EliminateCholesky);
  actualJacobianQR = *bt.marginalFactor(6, EliminateQRUnordered);
  //EXPECT(assert_equal(actualJacobianChol, actualJacobianQR)); // Check that Chol and QR obtained marginals are the same
  LONGS_EQUAL(2, (long)actualJacobianQR.rows());
  LONGS_EQUAL(1, (long)actualJacobianQR.size());
  LONGS_EQUAL(6, (long)actualJacobianQR.keys()[0]);
  actualA = actualJacobianQR.getA(actualJacobianQR.begin());
  actualCov = inverse(actualA.transpose() * actualA);
  EXPECT(assert_equal(expectedCov, actualCov, 1e1));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
