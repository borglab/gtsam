/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHessianFactor.cpp
 * @author  Richard Roberts
 * @date    Dec 15, 2010
 */

#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/assign/std/map.hpp>
using namespace boost::assign;

#include <vector>
#include <utility>

using namespace std;
using namespace gtsam;

const double tol = 1e-5;

/* ************************************************************************* */
TEST(HessianFactor, Slot)
{
  KeyVector keys {2, 4, 1};
  EXPECT_LONGS_EQUAL(0, GaussianFactor::Slot(keys,2));
  EXPECT_LONGS_EQUAL(1, GaussianFactor::Slot(keys,4));
  EXPECT_LONGS_EQUAL(2, GaussianFactor::Slot(keys,1));
  EXPECT_LONGS_EQUAL(3, GaussianFactor::Slot(keys,5)); // does not exist
}

/* ************************************************************************* */
TEST(HessianFactor, emptyConstructor)
{
  HessianFactor f;
  DOUBLES_EQUAL(0.0, f.constantTerm(), 1e-9);        // Constant term should be zero
  //EXPECT(assert_equal(Vector(), f.linearTerm()));    // Linear term should be empty
  EXPECT(assert_equal((Matrix) Z_1x1, f.info().selfadjointView())); // Full matrix should be 1-by-1 zero matrix
  DOUBLES_EQUAL(0.0, f.error(VectorValues()), 1e-9); // Should have zero error
}

/* ************************************************************************* */
TEST(HessianFactor, ConversionConstructor)
{
  HessianFactor expected(list_of(0)(1),
    SymmetricBlockMatrix(list_of(2)(4)(1), (Matrix(7,7) <<
      125.0000,       0.0,  -25.0000,       0.0, -100.0000,       0.0,   25.0000,
           0.0,  125.0000,       0.0,  -25.0000,       0.0, -100.0000,  -17.5000,
      -25.0000,       0.0,   25.0000,       0.0,       0.0,       0.0,   -5.0000,
           0.0,  -25.0000,       0.0,   25.0000,       0.0,       0.0,    7.5000,
     -100.0000,       0.0,       0.0,       0.0,  100.0000,       0.0,  -20.0000,
           0.0, -100.0000,       0.0,       0.0,       0.0,  100.0000,   10.0000,
       25.0000,  -17.5000,   -5.0000,    7.5000,  -20.0000,   10.0000,    8.2500).finished()));

  JacobianFactor jacobian(
    0, (Matrix(4,2) << -1., 0.,
                    +0.,-1.,
                     1., 0.,
                    +0.,1.).finished(),
    1, (Matrix(4,4) << 1., 0., 0.00,  0., // f4
                    0., 1., 0.00,  0., // f4
                    0., 0.,  -1.,  0., // f2
                    0., 0., 0.00, -1.).finished(), // f2
    (Vector(4) << -0.2, 0.3, 0.2, -0.1).finished(),
    noiseModel::Diagonal::Sigmas((Vector(4) << 0.2, 0.2, 0.1, 0.1).finished()));

  HessianFactor actual(jacobian);

  VectorValues values = pair_list_of<Key, Vector>
    (0, Vector2(1.0, 2.0))
    (1, (Vector(4) << 3.0, 4.0, 5.0, 6.0).finished());

  EXPECT_LONGS_EQUAL(2, (long)actual.size());
  EXPECT(assert_equal(expected, actual, 1e-9));
  EXPECT_DOUBLES_EQUAL(jacobian.error(values), actual.error(values), 1e-9);
}

/* ************************************************************************* */
TEST(HessianFactor, Constructor1)
{
  Matrix G = (Matrix(2,2) << 3.0, 5.0, 5.0, 6.0).finished();
  Vector g = Vector2(-8.0, -9.0);
  double f = 10.0;
  HessianFactor factor(0, G, g, f);

  // extract underlying parts
  EXPECT(assert_equal(G, Matrix(factor.info().diagonalBlock(0))));
  EXPECT_DOUBLES_EQUAL(f, factor.constantTerm(), 1e-10);
  EXPECT(assert_equal(g, Vector(factor.linearTerm())));
  EXPECT_LONGS_EQUAL(1, (long)factor.size());

  VectorValues dx = pair_list_of<Key, Vector>(0, Vector2(1.5, 2.5));

  // error 0.5*(f - 2*x'*g + x'*G*x)
  double expected = 80.375;
  double actual = factor.error(dx);
  double expected_manual = 0.5 * (f - 2.0 * dx[0].dot(g) + dx[0].transpose() * G.selfadjointView<Eigen::Upper>() * dx[0]);
  EXPECT_DOUBLES_EQUAL(expected, expected_manual, 1e-10);
  EXPECT_DOUBLES_EQUAL(expected, actual, 1e-10);
}

/* ************************************************************************* */
TEST(HessianFactor, Constructor1b)
{
  Vector mu = Vector2(1.0,2.0);
  Matrix Sigma = I_2x2;

  HessianFactor factor(0, mu, Sigma);

  Matrix G = I_2x2;
  Vector g = G*mu;
  double f = dot(g,mu);

  // Check
  EXPECT(assert_equal(G, Matrix(factor.info().diagonalBlock(0))));
  EXPECT_DOUBLES_EQUAL(f, factor.constantTerm(), 1e-10);
  EXPECT(assert_equal(g, Vector(factor.linearTerm())));
  EXPECT_LONGS_EQUAL(1, (long)factor.size());
}

/* ************************************************************************* */
TEST(HessianFactor, Constructor2)
{
  Matrix G11 = (Matrix(1,1) << 1.0).finished();
  Matrix G12 = (Matrix(1,2) << 2.0, 4.0).finished();
  Matrix G22 = (Matrix(2,2) << 3.0, 5.0, 5.0, 6.0).finished();
  Vector g1 = (Vector(1) << -7.0).finished();
  Vector g2 = Vector2(-8.0, -9.0);
  double f = 10.0;

  Vector dx0 = (Vector(1) << 0.5).finished();
  Vector dx1 = Vector2(1.5, 2.5);

  VectorValues dx = pair_list_of
    (0, dx0)
    (1, dx1);

  HessianFactor factor(0, 1, G11, G12, g1, G22, g2, f);

  double expected = 90.5;
  double actual = factor.error(dx);

  DOUBLES_EQUAL(expected, actual, 1e-10);
  LONGS_EQUAL(4, (long)factor.rows());
  DOUBLES_EQUAL(10.0, factor.constantTerm(), 1e-10);

  Vector linearExpected(3);  linearExpected << g1, g2;
  EXPECT(assert_equal(linearExpected, factor.linearTerm()));

  EXPECT(assert_equal(G11, factor.info().diagonalBlock(0)));
  EXPECT(assert_equal(G12, factor.info().aboveDiagonalBlock(0, 1)));
  EXPECT(assert_equal(G22, factor.info().diagonalBlock(1)));

  // Check case when vector values is larger than factor
  VectorValues dxLarge = pair_list_of<Key, Vector>
    (0, dx0)
    (1, dx1)
    (2, Vector2(0.1, 0.2));
  EXPECT_DOUBLES_EQUAL(expected, factor.error(dxLarge), 1e-10);
}

/* ************************************************************************* */
TEST(HessianFactor, Constructor3)
{
  Matrix G11 = (Matrix(1,1) << 1.0).finished();
  Matrix G12 = (Matrix(1,2) << 2.0, 4.0).finished();
  Matrix G13 = (Matrix(1,3) << 3.0, 6.0, 9.0).finished();

  Matrix G22 = (Matrix(2,2) << 3.0, 5.0, 5.0, 6.0).finished();
  Matrix G23 = (Matrix(2,3) << 4.0, 6.0, 8.0, 1.0, 2.0, 4.0).finished();

  Matrix G33 = (Matrix(3,3) << 1.0, 2.0, 3.0, 2.0, 5.0, 6.0, 3.0, 6.0, 9.0).finished();

  Vector g1 = (Vector(1) << -7.0).finished();
  Vector g2 = Vector2(-8.0, -9.0);
  Vector g3 = Vector3(1.0,  2.0,  3.0);

  double f = 10.0;

  Vector dx0 = (Vector(1) << 0.5).finished();
  Vector dx1 = Vector2(1.5, 2.5);
  Vector dx2 = Vector3(1.5, 2.5, 3.5);

  VectorValues dx = pair_list_of
    (0, dx0)
    (1, dx1)
    (2, dx2);

  HessianFactor factor(0, 1, 2, G11, G12, G13, g1, G22, G23, g2, G33, g3, f);

  double expected = 371.3750;
  double actual = factor.error(dx);

  DOUBLES_EQUAL(expected, actual, 1e-10);
  LONGS_EQUAL(7, (long)factor.rows());
  DOUBLES_EQUAL(10.0, factor.constantTerm(), 1e-10);

  Vector linearExpected(6);  linearExpected << g1, g2, g3;
  EXPECT(assert_equal(linearExpected, factor.linearTerm()));

  EXPECT(assert_equal(G11, factor.info().diagonalBlock(0)));
  EXPECT(assert_equal(G12, factor.info().aboveDiagonalBlock(0, 1)));
  EXPECT(assert_equal(G13, factor.info().aboveDiagonalBlock(0, 2)));
  EXPECT(assert_equal(G22, factor.info().diagonalBlock(1)));
  EXPECT(assert_equal(G23, factor.info().aboveDiagonalBlock(1, 2)));
  EXPECT(assert_equal(G33, factor.info().diagonalBlock(2)));
}

/* ************************************************************************* */
TEST(HessianFactor, ConstructorNWay)
{
  Matrix G11 = (Matrix(1,1) << 1.0).finished();
  Matrix G12 = (Matrix(1,2) << 2.0, 4.0).finished();
  Matrix G13 = (Matrix(1,3) << 3.0, 6.0, 9.0).finished();

  Matrix G22 = (Matrix(2,2) << 3.0, 5.0, 5.0, 6.0).finished();
  Matrix G23 = (Matrix(2,3) << 4.0, 6.0, 8.0, 1.0, 2.0, 4.0).finished();

  Matrix G33 = (Matrix(3,3) << 1.0, 2.0, 3.0, 2.0, 5.0, 6.0, 3.0, 6.0, 9.0).finished();

  Vector g1 = (Vector(1) << -7.0).finished();
  Vector g2 = Vector2(-8.0, -9.0);
  Vector g3 = Vector3(1.0,  2.0,  3.0);

  double f = 10.0;

  Vector dx0 = (Vector(1) << 0.5).finished();
  Vector dx1 = Vector2(1.5, 2.5);
  Vector dx2 = Vector3(1.5, 2.5, 3.5);

  VectorValues dx = pair_list_of
    (0, dx0)
    (1, dx1)
    (2, dx2);

  KeyVector js {0, 1, 2};
  std::vector<Matrix> Gs;
  Gs.push_back(G11); Gs.push_back(G12); Gs.push_back(G13); Gs.push_back(G22); Gs.push_back(G23); Gs.push_back(G33);
  std::vector<Vector> gs;
  gs.push_back(g1); gs.push_back(g2); gs.push_back(g3);
  HessianFactor factor(js, Gs, gs, f);

  double expected = 371.3750;
  double actual = factor.error(dx);

  DOUBLES_EQUAL(expected, actual, 1e-10);
  LONGS_EQUAL(7, (long)factor.rows());
  DOUBLES_EQUAL(10.0, factor.constantTerm(), 1e-10);

  Vector linearExpected(6);  linearExpected << g1, g2, g3;
  EXPECT(assert_equal(linearExpected, factor.linearTerm()));

  EXPECT(assert_equal(G11, factor.info().diagonalBlock(0)));
  EXPECT(assert_equal(G12, factor.info().aboveDiagonalBlock(0, 1)));
  EXPECT(assert_equal(G13, factor.info().aboveDiagonalBlock(0, 2)));
  EXPECT(assert_equal(G22, factor.info().diagonalBlock(1)));
  EXPECT(assert_equal(G23, factor.info().aboveDiagonalBlock(1, 2)));
  EXPECT(assert_equal(G33, factor.info().diagonalBlock(2)));
}

/* ************************************************************************* */
TEST(HessianFactor, CombineAndEliminate1) {
  Matrix3 A01 = 3.0 * I_3x3;
  Vector3 b0(1, 0, 0);

  Matrix3 A21 = 4.0 * I_3x3;
  Vector3 b2 = Vector3::Zero();

  GaussianFactorGraph gfg;
  gfg.add(1, A01, b0);
  gfg.add(1, A21, b2);

  Matrix63 A1;
  A1 << A01, A21;
  Vector6 b;
  b << b0, b2;

  // create a full, uneliminated version of the factor
  JacobianFactor jacobian(1, A1, b);

  // Make sure combining works
  HessianFactor hessian(gfg);
  VectorValues v;
  v.insert(1, Vector3(1, 0, 0));
  EXPECT_DOUBLES_EQUAL(jacobian.error(v), hessian.error(v), 1e-9);
  EXPECT(assert_equal(HessianFactor(jacobian), hessian, 1e-6));
  EXPECT(assert_equal(25.0 * I_3x3, hessian.information(), 1e-9));
  EXPECT(
      assert_equal(jacobian.augmentedInformation(),
          hessian.augmentedInformation(), 1e-9));

  // perform elimination on jacobian
  Ordering ordering = list_of(1);
  GaussianConditional::shared_ptr expectedConditional;
  JacobianFactor::shared_ptr expectedFactor;
  boost::tie(expectedConditional, expectedFactor) = jacobian.eliminate(ordering);
  CHECK(expectedFactor);

  // Eliminate
  GaussianConditional::shared_ptr actualConditional;
  HessianFactor::shared_ptr actualHessian;
  boost::tie(actualConditional, actualHessian) = //
      EliminateCholesky(gfg, ordering);
  actualConditional->setModel(false,Vector3(1,1,1)); // add a unit model for comparison

  EXPECT(assert_equal(*expectedConditional, *actualConditional, 1e-6));
#ifdef TEST_ERROR_EQUIVALENCE
  // these tests disabled because QR cuts off the all zeros + error row
  EXPECT_DOUBLES_EQUAL(expectedFactor->error(v), actualHessian->error(v), 1e-9);
  EXPECT(
      assert_equal(expectedFactor->augmentedInformation(),
          actualHessian->augmentedInformation(), 1e-9));
  EXPECT(assert_equal(HessianFactor(*expectedFactor), *actualHessian, 1e-6));
#endif
  }

/* ************************************************************************* */
TEST(HessianFactor, CombineAndEliminate2) {
  Matrix A01 = I_3x3;
  Vector3 b0(1.5, 1.5, 1.5);
  Vector3 s0(1.6, 1.6, 1.6);

  Matrix A10 = 2.0 * I_3x3;
  Matrix A11 = -2.0 * I_3x3;
  Vector3 b1(2.5, 2.5, 2.5);
  Vector3 s1(2.6, 2.6, 2.6);

  Matrix A21 = 3.0 * I_3x3;
  Vector3 b2(3.5, 3.5, 3.5);
  Vector3 s2(3.6, 3.6, 3.6);

  GaussianFactorGraph gfg;
  gfg.add(1, A01, b0, noiseModel::Diagonal::Sigmas(s0, true));
  gfg.add(0, A10, 1, A11, b1, noiseModel::Diagonal::Sigmas(s1, true));
  gfg.add(1, A21, b2, noiseModel::Diagonal::Sigmas(s2, true));

  Matrix93 A0, A1;
  A0 << A10, Z_3x3, Z_3x3;
  A1 << A11, A01, A21;
  Vector9 b, sigmas;
  b << b1, b0, b2;
  sigmas << s1, s0, s2;

  // create a full, uneliminated version of the factor
  JacobianFactor jacobian(0, A0, 1, A1, b,
      noiseModel::Diagonal::Sigmas(sigmas, true));

  // Make sure combining works
  HessianFactor hessian(gfg);
  EXPECT(assert_equal(HessianFactor(jacobian), hessian, 1e-6));
  EXPECT(
      assert_equal(jacobian.augmentedInformation(),
          hessian.augmentedInformation(), 1e-9));

  // perform elimination on jacobian
  Ordering ordering = list_of(0);
  GaussianConditional::shared_ptr expectedConditional;
  JacobianFactor::shared_ptr expectedFactor;
  boost::tie(expectedConditional, expectedFactor) = //
      jacobian.eliminate(ordering);

  // Eliminate
  GaussianConditional::shared_ptr actualConditional;
  HessianFactor::shared_ptr actualHessian;
  boost::tie(actualConditional, actualHessian) = //
      EliminateCholesky(gfg, ordering);
  actualConditional->setModel(false,Vector3(1,1,1)); // add a unit model for comparison

  EXPECT(assert_equal(*expectedConditional, *actualConditional, 1e-6));
#ifdef TEST_ERROR_EQUIVALENCE
  // these tests disabled because QR cuts off the all zeros + error row
  VectorValues v;
  v.insert(1, Vector3(1, 2, 3));
  EXPECT_DOUBLES_EQUAL(expectedFactor->error(v), actualHessian->error(v), 1e-9);
  EXPECT(
      assert_equal(expectedFactor->augmentedInformation(),
          actualHessian->augmentedInformation(), 1e-9));
  EXPECT(assert_equal(HessianFactor(*expectedFactor), *actualHessian, 1e-6));
#endif
}

/* ************************************************************************* */
TEST(HessianFactor, eliminate2 )
{
  // sigmas
  double sigma1 = 0.2;
  double sigma2 = 0.1;
  Vector sigmas = (Vector(4) << sigma1, sigma1, sigma2, sigma2).finished();

  // the combined linear factor
  Matrix Ax2 = (Matrix(4,2) <<
      // x2
      -1., 0.,
      +0.,-1.,
      1., 0.,
      +0.,1.
  ).finished();

  Matrix Al1x1 = (Matrix(4,4) <<
      // l1   x1
      1., 0., 0.00,  0., // f4
      0., 1., 0.00,  0., // f4
      0., 0., -1.,  0., // f2
      0., 0., 0.00,-1.  // f2
  ).finished();

  // the RHS
  Vector b2(4);
  b2(0) = -0.2;
  b2(1) =  0.3;
  b2(2) =  0.2;
  b2(3) = -0.1;

  vector<pair<Key, Matrix> > meas;
  meas.push_back(make_pair(0, Ax2));
  meas.push_back(make_pair(1, Al1x1));
  JacobianFactor combined(meas, b2, noiseModel::Diagonal::Sigmas(sigmas));

  // eliminate the combined factor
  HessianFactor::shared_ptr combinedLF_Chol(new HessianFactor(combined));
  GaussianFactorGraph combinedLFG_Chol = list_of(combinedLF_Chol);

  std::pair<GaussianConditional::shared_ptr, HessianFactor::shared_ptr> actual_Chol =
    EliminateCholesky(combinedLFG_Chol, Ordering(list_of(0)));

  // create expected Conditional Gaussian
  double oldSigma = 0.0894427; // from when R was made unit
  Matrix R11 = (Matrix(2,2) <<
      1.00,  0.00,
      0.00,  1.00
  ).finished()/oldSigma;
  Matrix S12 = (Matrix(2,4) <<
      -0.20, 0.00,-0.80, 0.00,
      +0.00,-0.20,+0.00,-0.80
  ).finished()/oldSigma;
  Vector d = Vector2(0.2,-0.14)/oldSigma;
  GaussianConditional expectedCG(0, d, R11, 1, S12);
  EXPECT(assert_equal(expectedCG, *actual_Chol.first, 1e-4));

  // the expected linear factor
  double sigma = 0.2236;
  Matrix Bl1x1 = (Matrix(2,4) <<
      // l1          x1
      1.00, 0.00, -1.00,  0.00,
      0.00, 1.00, +0.00, -1.00
  ).finished()/sigma;
  Vector b1 = Vector2(0.0,0.894427);
  JacobianFactor expectedLF(1, Bl1x1, b1, noiseModel::Isotropic::Sigma(2,1.0));
  EXPECT(assert_equal(HessianFactor(expectedLF), *actual_Chol.second, 1.5e-3));
}

/* ************************************************************************* */
TEST(HessianFactor, combine) {

  // update the information matrix with a single jacobian factor
  Matrix A0 = (Matrix(2, 2) <<
  11.1803399,     0.0,
      0.0, 11.1803399).finished();
  Matrix A1 = (Matrix(2, 2) <<
  -2.23606798,        0.0,
         0.0, -2.23606798).finished();
  Matrix A2 = (Matrix(2, 2) <<
  -8.94427191,      0.0,
         0.0, -8.94427191).finished();
  Vector b = Vector2(2.23606798,-1.56524758);
  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector::Ones(2));
  GaussianFactor::shared_ptr f(new JacobianFactor(0, A0, 1, A1, 2, A2, b, model));
  GaussianFactorGraph factors = list_of(f);

  // Form Ab' * Ab
  HessianFactor actual(factors);

  Matrix expected = (Matrix(7, 7) <<
  125.0000,       0.0,  -25.0000,       0.0, -100.0000,       0.0,   25.0000,
       0.0,  125.0000,       0.0,  -25.0000,       0.0, -100.0000,  -17.5000,
  -25.0000,       0.0,    5.0000,       0.0,   20.0000,       0.0,   -5.0000,
       0.0,  -25.0000,       0.0,    5.0000,       0.0,   20.0000,    3.5000,
 -100.0000,       0.0,   20.0000,       0.0,   80.0000,       0.0,  -20.0000,
       0.0, -100.0000,       0.0,   20.0000,       0.0,   80.0000,   14.0000,
   25.0000,  -17.5000,   -5.0000,    3.5000,  -20.0000,   14.0000,    7.4500).finished();
  EXPECT(assert_equal(expected, Matrix(actual.info().selfadjointView()), tol));

}

/* ************************************************************************* */
TEST(HessianFactor, gradientAtZero)
{
  Matrix G11 = (Matrix(1, 1) << 1).finished();
  Matrix G12 = (Matrix(1, 2) << 0, 0).finished();
  Matrix G22 = (Matrix(2, 2) << 1, 0, 0, 1).finished();
  Vector g1 = (Vector(1) << -7).finished();
  Vector g2 = Vector2(-8, -9);
  double f = 194;

  HessianFactor factor(0, 1, G11, G12, g1, G22, g2, f);

  // test gradient at zero
  VectorValues expectedG = pair_list_of<Key, Vector>(0, -g1) (1, -g2);
  Matrix A; Vector b; boost::tie(A,b) = factor.jacobian();
  KeyVector keys {0, 1};
  EXPECT(assert_equal(-A.transpose()*b, expectedG.vector(keys)));
  VectorValues actualG = factor.gradientAtZero();
  EXPECT(assert_equal(expectedG, actualG));
}

/* ************************************************************************* */
TEST(HessianFactor, gradient)
{
  Matrix G11 = (Matrix(1, 1) << 1).finished();
  Matrix G12 = (Matrix(1, 2) << 0, 0).finished();
  Matrix G22 = (Matrix(2, 2) << 1, 0, 0, 1).finished();
  Vector g1 = (Vector(1) << -7).finished();
  Vector g2 = (Vector(2) << -8, -9).finished();
  double f = 194;

  HessianFactor factor(0, 1, G11, G12, g1, G22, g2, f);

  // test gradient
  Vector x0 = (Vector(1) << 3.0).finished();
  Vector x1 = (Vector(2) << -3.5, 7.1).finished();
  VectorValues x = pair_list_of<Key, Vector>(0, x0) (1, x1);

  Vector expectedGrad0 = (Vector(1) << 10.0).finished();
  Vector expectedGrad1 = (Vector(2) << 4.5, 16.1).finished();
  Vector grad0 = factor.gradient(0, x);
  Vector grad1 = factor.gradient(1, x);

  EXPECT(assert_equal(expectedGrad0, grad0));
  EXPECT(assert_equal(expectedGrad1, grad1));
}

/* ************************************************************************* */
TEST(HessianFactor, hessianDiagonal)
{
  Matrix G11 = (Matrix(1, 1) << 1).finished();
  Matrix G12 = (Matrix(1, 2) << 0, 0).finished();
  Matrix G22 = (Matrix(2, 2) << 1, 0, 0, 1).finished();
  Vector g1 = (Vector(1) << -7).finished();
  Vector g2 = Vector2(-8, -9);
  double f = 194;

  HessianFactor factor(0, 1, G11, G12, g1, G22, g2, f);

  // hessianDiagonal
  VectorValues expected;
  expected.insert(0, (Vector(1) << 1).finished());
  expected.insert(1, Vector2(1,1));
  EXPECT(assert_equal(expected, factor.hessianDiagonal()));

  // hessianBlockDiagonal
  map<Key,Matrix> actualBD = factor.hessianBlockDiagonal();
  LONGS_EQUAL(2,actualBD.size());
  EXPECT(assert_equal(G11,actualBD[0]));
  EXPECT(assert_equal(G22,actualBD[1]));
}

/* ************************************************************************* */
TEST(HessianFactor, Solve)
{
  Matrix2 A;
  A << 1, 2, 3, 4;
  Matrix2 G = A.transpose() * A;
  Vector2 b(5, 6);
  Vector2 g = A.transpose() * b;
  double f = 0;
  Key key(55);
  HessianFactor factor(key, G, g, f);

  VectorValues expected;
  expected.insert(key, A.inverse() * b);
  EXPECT(assert_equal(expected, factor.solve()));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
