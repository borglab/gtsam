/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testJacobianFactor.cpp
 *  @brief  Unit tests for Linear Factor
 *  @author Christian Potthast
 *  @author Frank Dellaert
 **/

#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/VariableSlots.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>

#include <boost/range/iterator_range.hpp>

using namespace std;
using namespace gtsam;

using Dims = std::vector<Eigen::Index>;  // For constructing block matrices

namespace {
  namespace simple {
    // Terms we'll use
  using Terms = vector<pair<Key, Matrix> >;
  const Terms terms{{5, I_3x3}, {10, 2 * I_3x3}, {15, 3 * I_3x3}};

  // RHS and sigmas
  const Vector b = Vector3(1., 2., 3.);
  const SharedDiagonal noise =
      noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.5));
  }
}

/* ************************************************************************* */
TEST(JacobianFactor, constructors_and_accessors)
{
  using namespace simple;

  // Test for using different numbers of terms
  {
    // b vector only constructor
    JacobianFactor expected(Terms{}, b);
    JacobianFactor actual(b);
    EXPECT(assert_equal(expected, actual));
    EXPECT(assert_equal(b, expected.getb()));
    EXPECT(assert_equal(b, actual.getb()));
    EXPECT(!expected.get_model());
    EXPECT(!actual.get_model());
  }
  {
    // One term constructor
    JacobianFactor expected(Terms{terms[0]}, b, noise);
    JacobianFactor actual(terms[0].first, terms[0].second, b, noise);
    EXPECT(assert_equal(expected, actual));
    LONGS_EQUAL((long)terms[0].first, (long)actual.keys().back());
    EXPECT(assert_equal(terms[0].second, actual.getA(actual.end() - 1)));
    EXPECT(assert_equal(b, expected.getb()));
    EXPECT(assert_equal(b, actual.getb()));
    EXPECT(noise == expected.get_model());
    EXPECT(noise == actual.get_model());
  }
  {
    // Two term constructor
    JacobianFactor expected(Terms{terms[0], terms[1]}, b, noise);
    JacobianFactor actual(terms[0].first, terms[0].second,
      terms[1].first, terms[1].second, b, noise);
    EXPECT(assert_equal(expected, actual));
    LONGS_EQUAL((long)terms[1].first, (long)actual.keys().back());
    EXPECT(assert_equal(terms[1].second, actual.getA(actual.end() - 1)));
    EXPECT(assert_equal(b, expected.getb()));
    EXPECT(assert_equal(b, actual.getb()));
    EXPECT(noise == expected.get_model());
    EXPECT(noise == actual.get_model());
  }
  {
    // Three term constructor
    JacobianFactor expected(Terms{terms[0], terms[1], terms[2]}, b, noise);
    JacobianFactor actual(terms[0].first, terms[0].second,
      terms[1].first, terms[1].second, terms[2].first, terms[2].second, b, noise);
    EXPECT(assert_equal(expected, actual));
    LONGS_EQUAL((long)terms[2].first, (long)actual.keys().back());
    EXPECT(assert_equal(terms[2].second, actual.getA(actual.end() - 1)));
    EXPECT(assert_equal(b, expected.getb()));
    EXPECT(assert_equal(b, actual.getb()));
    EXPECT(noise == expected.get_model());
    EXPECT(noise == actual.get_model());
  }
  {
    // Test three-term constructor with std::map
    JacobianFactor expected(Terms{terms[0], terms[1], terms[2]}, b, noise);
    map<Key,Matrix> mapTerms;
    // note order of insertion plays no role: order will be determined by keys
    mapTerms.insert(terms[2]);
    mapTerms.insert(terms[1]);
    mapTerms.insert(terms[0]);
    JacobianFactor actual(mapTerms, b, noise);
    EXPECT(assert_equal(expected, actual));
    LONGS_EQUAL((long)terms[2].first, (long)actual.keys().back());
    EXPECT(assert_equal(terms[2].second, actual.getA(actual.end() - 1)));
    EXPECT(assert_equal(b, expected.getb()));
    EXPECT(assert_equal(b, actual.getb()));
    EXPECT(noise == expected.get_model());
    EXPECT(noise == actual.get_model());
  }
  {
    // VerticalBlockMatrix constructor
    JacobianFactor expected(Terms{terms[0], terms[1], terms[2]}, b, noise);
    VerticalBlockMatrix blockMatrix(Dims{3, 3, 3, 1}, 3);
    blockMatrix(0) = terms[0].second;
    blockMatrix(1) = terms[1].second;
    blockMatrix(2) = terms[2].second;
    blockMatrix(3) = b;
    // get a vector of keys from the terms
    vector<Key> keys;
    for (const auto& term : terms)
      keys.push_back(term.first);
    JacobianFactor actual(keys, blockMatrix, noise);
    EXPECT(assert_equal(expected, actual));
    LONGS_EQUAL((long)terms[2].first, (long)actual.keys().back());
    EXPECT(assert_equal(terms[2].second, actual.getA(actual.end() - 1)));
    EXPECT(assert_equal(b, expected.getb()));
    EXPECT(assert_equal(b, actual.getb()));
    EXPECT(noise == expected.get_model());
    EXPECT(noise == actual.get_model());
  }
}

/* ************************************************************************* */
TEST(JabobianFactor, Hessian_conversion) {
  HessianFactor hessian(0, (Matrix(4, 4) <<
        1.57,        2.695,         -1.1,        -2.35,
       2.695,      11.3125,        -0.65,      -10.225,
        -1.1,        -0.65,            1,          0.5,
       -2.35,      -10.225,          0.5,         9.25).finished(),
      (Vector(4) << -7.885, -28.5175, 2.75, 25.675).finished(),
      73.1725);

  JacobianFactor expected(0, (Matrix(2, 4) <<
      1.2530,   2.1508,   -0.8779,  -1.8755,
           0,   2.5858,    0.4789,  -2.3943).finished(),
      Vector2(-6.2929, -5.7941));

  EXPECT(assert_equal(expected, JacobianFactor(hessian), 1e-3));
}

/* ************************************************************************* */
TEST(JabobianFactor, Hessian_conversion2) {
  JacobianFactor jf(0, (Matrix(3, 3) <<
      1, 2, 3,
      0, 2, 3,
      0, 0, 3).finished(),
    Vector3(1, 2, 2));
  HessianFactor hessian(jf);
  EXPECT(assert_equal(jf, JacobianFactor(hessian), 1e-9));
}

/* ************************************************************************* */
TEST(JabobianFactor, Hessian_conversion3) {
  JacobianFactor jf(0, (Matrix(2, 4) <<
      1, 2, 3, 0,
      0, 3, 2, 1).finished(),
    Vector2(1, 2));
  HessianFactor hessian(jf);
  EXPECT(assert_equal(jf, JacobianFactor(hessian), 1e-9));
}

/* ************************************************************************* */
namespace simple_graph {

Key keyX(10), keyY(8), keyZ(12);

double sigma1 = 0.1;
Matrix A11 = I_2x2;
Vector2 b1(2, -1);
auto factor1 = std::make_shared<JacobianFactor>(
    keyX, A11, b1, noiseModel::Isotropic::Sigma(2, sigma1));

double sigma2 = 0.5;
Matrix A21 = -2 * I_2x2;
Matrix A22 = 3 * I_2x2;
Vector2 b2(4, -5);
auto factor2 = std::make_shared<JacobianFactor>(
    keyX, A21, keyY, A22, b2, noiseModel::Isotropic::Sigma(2, sigma2));

double sigma3 = 1.0;
Matrix A32 = -4 * I_2x2;
Matrix A33 = 5 * I_2x2;
Vector2 b3(3, -6);
auto factor3 = std::make_shared<JacobianFactor>(
    keyY, A32, keyZ, A33, b3, noiseModel::Isotropic::Sigma(2, sigma3));

const GaussianFactorGraph factors { factor1, factor2, factor3 };
const Ordering ordering { keyX, keyY, keyZ };
}

/* ************************************************************************* */
TEST( JacobianFactor, construct_from_graph)
{
  using namespace simple_graph;

  Matrix A1(6,2); A1 << A11, A21, Z_2x2;
  Matrix A2(6,2); A2 << Z_2x2, A22, A32;
  Matrix A3(6,2); A3 << Matrix::Zero(4,2), A33;
  Vector b(6); b << b1, b2, b3;
  Vector sigmas(6); sigmas << sigma1, sigma1, sigma2, sigma2, sigma3, sigma3;
  JacobianFactor expected(keyX, A1, keyY, A2, keyZ, A3, b, noiseModel::Diagonal::Sigmas(sigmas));

  // The ordering here specifies the order in which the variables will appear in the combined factor
  JacobianFactor actual(factors, ordering);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(JacobianFactor, error)
{
  JacobianFactor factor(simple::terms, simple::b, simple::noise);

  VectorValues values;
  values.insert(5, Vector::Constant(3, 1.0));
  values.insert(10, Vector::Constant(3, 0.5));
  values.insert(15, Vector::Constant(3, 1.0/3.0));

  Vector expected_unwhitened(3); expected_unwhitened << 2.0, 1.0, 0.0;
  Vector actual_unwhitened = factor.unweighted_error(values);
  EXPECT(assert_equal(expected_unwhitened, actual_unwhitened));

  Vector expected_whitened(3); expected_whitened << 4.0, 2.0, 0.0;
  Vector actual_whitened = factor.error_vector(values);
  EXPECT(assert_equal(expected_whitened, actual_whitened));

  double expected_error = 0.5 * expected_whitened.squaredNorm();
  double actual_error = factor.error(values);
  DOUBLES_EQUAL(expected_error, actual_error, 1e-10);
}

/* ************************************************************************* */
TEST(JacobianFactor, matrices_NULL)
{
  // Make sure everything works with nullptr noise model
  JacobianFactor factor(simple::terms, simple::b);

  Matrix jacobianExpected(3, 9);
  jacobianExpected << simple::terms[0].second, simple::terms[1].second, simple::terms[2].second;
  Vector rhsExpected = simple::b;
  Matrix augmentedJacobianExpected(3, 10);
  augmentedJacobianExpected << jacobianExpected, rhsExpected;

  Matrix augmentedHessianExpected =
    augmentedJacobianExpected.transpose() * augmentedJacobianExpected;

  // Hessian
  EXPECT(assert_equal(Matrix(augmentedHessianExpected.topLeftCorner(9,9)), factor.information()));
  EXPECT(assert_equal(augmentedHessianExpected, factor.augmentedInformation()));

  // Whitened Jacobian
  EXPECT(assert_equal(jacobianExpected, factor.jacobian().first));
  EXPECT(assert_equal(rhsExpected, factor.jacobian().second));
  EXPECT(assert_equal(augmentedJacobianExpected, factor.augmentedJacobian()));

  // Unwhitened Jacobian
  EXPECT(assert_equal(jacobianExpected, factor.jacobianUnweighted().first));
  EXPECT(assert_equal(rhsExpected, factor.jacobianUnweighted().second));
  EXPECT(assert_equal(augmentedJacobianExpected, factor.augmentedJacobianUnweighted()));

  // hessianDiagonal
  VectorValues expectDiagonal;
  expectDiagonal.insert(5, Vector::Ones(3));
  expectDiagonal.insert(10, 4*Vector::Ones(3));
  expectDiagonal.insert(15, 9*Vector::Ones(3));
  EXPECT(assert_equal(expectDiagonal, factor.hessianDiagonal()));

  // hessianBlockDiagonal
  map<Key,Matrix> actualBD = factor.hessianBlockDiagonal();
  LONGS_EQUAL(3,actualBD.size());
  EXPECT(assert_equal(1*I_3x3,actualBD[5]));
  EXPECT(assert_equal(4*I_3x3,actualBD[10]));
  EXPECT(assert_equal(9*I_3x3,actualBD[15]));
}

/* ************************************************************************* */
TEST(JacobianFactor, matrices)
{
  // And now witgh a non-unit noise model
  JacobianFactor factor(simple::terms, simple::b, simple::noise);

  Matrix jacobianExpected(3, 9);
  jacobianExpected << simple::terms[0].second, simple::terms[1].second, simple::terms[2].second;
  Vector rhsExpected = simple::b;
  Matrix augmentedJacobianExpected(3, 10);
  augmentedJacobianExpected << jacobianExpected, rhsExpected;

  Matrix augmentedHessianExpected =
    augmentedJacobianExpected.transpose() * simple::noise->R().transpose()
    * simple::noise->R() * augmentedJacobianExpected;

  // Hessian
  EXPECT(assert_equal(Matrix(augmentedHessianExpected.topLeftCorner(9,9)), factor.information()));
  EXPECT(assert_equal(augmentedHessianExpected, factor.augmentedInformation()));

  // Whitened Jacobian
  EXPECT(assert_equal(simple::noise->R() * jacobianExpected, factor.jacobian().first));
  EXPECT(assert_equal(simple::noise->R() * rhsExpected, factor.jacobian().second));
  EXPECT(assert_equal(simple::noise->R() * augmentedJacobianExpected, factor.augmentedJacobian()));

  // Unwhitened Jacobian
  EXPECT(assert_equal(jacobianExpected, factor.jacobianUnweighted().first));
  EXPECT(assert_equal(rhsExpected, factor.jacobianUnweighted().second));
  EXPECT(assert_equal(augmentedJacobianExpected, factor.augmentedJacobianUnweighted()));

  // hessianDiagonal
  VectorValues expectDiagonal;
  // below we divide by the variance 0.5^2
  expectDiagonal.insert(5, Vector3(1, 1, 1)/0.25);
  expectDiagonal.insert(10, Vector3(4, 4, 4)/0.25);
  expectDiagonal.insert(15, Vector3(9, 9, 9)/0.25);
  EXPECT(assert_equal(expectDiagonal, factor.hessianDiagonal()));

  // hessianBlockDiagonal
  map<Key,Matrix> actualBD = factor.hessianBlockDiagonal();
  LONGS_EQUAL(3,actualBD.size());
  EXPECT(assert_equal(4*I_3x3,actualBD[5]));
  EXPECT(assert_equal(16*I_3x3,actualBD[10]));
  EXPECT(assert_equal(36*I_3x3,actualBD[15]));
}

/* ************************************************************************* */
TEST(JacobianFactor, operators )
{
  const double sigma = 0.1;
  SharedDiagonal sigma0_1 = noiseModel::Isotropic::Sigma(2, sigma);

  Matrix I = I_2x2;
  Vector b = Vector2(0.2,-0.1);
  JacobianFactor lf(1, -I, 2, I, b, sigma0_1);

  VectorValues x;
  Vector2 x1(10,20), x2(30,60);
  x.insert(1, x1);
  x.insert(2, x2);

  // test A*x
  Vector expectedE = (x2 - x1)/sigma;
  Vector actualE = lf * x;
  EXPECT(assert_equal(expectedE, actualE));

  // test A^e
  VectorValues expectedX;
  const double alpha = 0.5;
  expectedX.insert(1, - alpha * expectedE /sigma);
  expectedX.insert(2,   alpha * expectedE /sigma);
  VectorValues actualX = VectorValues::Zero(expectedX);
  lf.transposeMultiplyAdd(alpha, actualE, actualX);
  EXPECT(assert_equal(expectedX, actualX));

  // test gradient at zero
  const auto [A, b2] = lf.jacobian();
  VectorValues expectedG;
  expectedG.insert(1, Vector2(20,-10));
  expectedG.insert(2, Vector2(-20, 10));
  KeyVector keys {1, 2};
  EXPECT(assert_equal(-A.transpose()*b2, expectedG.vector(keys)));
  VectorValues actualG = lf.gradientAtZero();
  EXPECT(assert_equal(expectedG, actualG));
}

/* ************************************************************************* */
TEST(JacobianFactor, default_error )
{
  JacobianFactor f;
  double actual = f.error(VectorValues());
  DOUBLES_EQUAL(0.0, actual, 1e-15);
}

//* ************************************************************************* */
TEST(JacobianFactor, empty )
{
  // create an empty factor
  JacobianFactor f;
  EXPECT(f.empty());
}

/* ************************************************************************* */
TEST(JacobianFactor, eliminate)
{
  Matrix A01 = (Matrix(3, 3) <<
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0).finished();
  Vector3 b0(1.5, 1.5, 1.5);
  Vector3 s0(1.6, 1.6, 1.6);

  Matrix A10 = (Matrix(3, 3) <<
    2.0, 0.0, 0.0,
    0.0, 2.0, 0.0,
    0.0, 0.0, 2.0).finished();
  Matrix A11 = (Matrix(3, 3) <<
    -2.0, 0.0, 0.0,
    0.0, -2.0, 0.0,
    0.0, 0.0, -2.0).finished();
  Vector3 b1(2.5, 2.5, 2.5);
  Vector3 s1(2.6, 2.6, 2.6);

  Matrix A21 = (Matrix(3, 3) <<
    3.0, 0.0, 0.0,
    0.0, 3.0, 0.0,
    0.0, 0.0, 3.0).finished();
  Vector3 b2(3.5, 3.5, 3.5);
  Vector3 s2(3.6, 3.6, 3.6);

  GaussianFactorGraph gfg;
  gfg.add(1, A01, b0, noiseModel::Diagonal::Sigmas(s0, true));
  gfg.add(0, A10, 1, A11, b1, noiseModel::Diagonal::Sigmas(s1, true));
  gfg.add(1, A21, b2, noiseModel::Diagonal::Sigmas(s2, true));

  Matrix zero3x3 = Matrix::Zero(3,3);
  Matrix A0 = gtsam::stack(3, &A10, &zero3x3, &zero3x3);
  Matrix A1 = gtsam::stack(3, &A11, &A01, &A21);
  Vector9 b; b << b1, b0, b2;
  Vector9 sigmas; sigmas << s1, s0, s2;

  JacobianFactor combinedFactor(0, A0, 1, A1, b, noiseModel::Diagonal::Sigmas(sigmas, true));
  GaussianFactorGraph::EliminationResult expected = combinedFactor.eliminate(Ordering{0});
  JacobianFactor::shared_ptr expectedJacobian = std::dynamic_pointer_cast<
    JacobianFactor>(expected.second);

  GaussianFactorGraph::EliminationResult actual = EliminateQR(gfg, Ordering{0});
  JacobianFactor::shared_ptr actualJacobian = std::dynamic_pointer_cast<
    JacobianFactor>(actual.second);

  EXPECT(assert_equal(*expected.first, *actual.first));
  EXPECT(assert_equal(*expectedJacobian, *actualJacobian));
}

/* ************************************************************************* */
TEST(JacobianFactor, eliminate2 )
{
  // sigmas
  double sigma1 = 0.2;
  double sigma2 = 0.1;
  Vector sigmas = (Vector(4) << sigma1, sigma1, sigma2, sigma2).finished();

  // the combined linear factor
  Matrix Ax2 = (Matrix(4, 2) <<
    // x2
    -1., 0.,
    +0.,-1.,
    1., 0.,
    +0.,1.
    ).finished();

  Matrix Al1x1 = (Matrix(4, 4) <<
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
  meas.push_back(make_pair(2, Ax2));
  meas.push_back(make_pair(11, Al1x1));
  JacobianFactor combined(meas, b2, noiseModel::Diagonal::Sigmas(sigmas));

  // eliminate the combined factor
  pair<GaussianConditional::shared_ptr, JacobianFactor::shared_ptr>
    actual = combined.eliminate(Ordering{2});

  // create expected Conditional Gaussian
  double oldSigma = 0.0894427; // from when R was made unit
  Matrix R11 = (Matrix(2, 2) <<
    1.00,  0.00,
    0.00,  1.00
    ).finished()/oldSigma;
  Matrix S12 = (Matrix(2, 4) <<
    -0.20, 0.00,-0.80, 0.00,
    +0.00,-0.20,+0.00,-0.80
    ).finished()/oldSigma;
  Vector d = Vector2(0.2,-0.14)/oldSigma;
  GaussianConditional expectedCG(2, d, R11, 11, S12, noiseModel::Unit::Create(2));

  EXPECT(assert_equal(expectedCG, *actual.first, 1e-4));

  // the expected linear factor
  double sigma = 0.2236;
  Matrix Bl1x1 = (Matrix(2, 4) <<
    // l1          x1
    1.00, 0.00, -1.00,  0.00,
    0.00, 1.00, +0.00, -1.00
    ).finished()/sigma;
  Vector b1 = Vector2(0.0, 0.894427);
  JacobianFactor expectedLF(11, Bl1x1, b1, noiseModel::Unit::Create(2));
  EXPECT(assert_equal(expectedLF, *actual.second,1e-3));
}

/* ************************************************************************* */
TEST(JacobianFactor, EliminateQR)
{
  // Augmented Ab test case for whole factor graph
  Matrix Ab = (Matrix(14, 11) <<
    4.,     0.,     1.,     4.,     1.,     0.,     3.,     6.,     8.,     8.,     1.,
    9.,     2.,     0.,     1.,     6.,     3.,     9.,     6.,     6.,     9.,     4.,
    5.,     3.,     7.,     9.,     5.,     5.,     9.,     1.,     3.,     7.,     0.,
    5.,     6.,     5.,     7.,     9.,     4.,     0.,     1.,     1.,     3.,     5.,
    0.,     0.,     4.,     5.,     6.,     6.,     7.,     9.,     4.,     5.,     4.,
    0.,     0.,     9.,     4.,     8.,     6.,     2.,     1.,     4.,     1.,     6.,
    0.,     0.,     6.,     0.,     4.,     2.,     4.,     0.,     1.,     9.,     6.,
    0.,     0.,     6.,     6.,     4.,     4.,     5.,     5.,     5.,     8.,     6.,
    0.,     0.,     0.,     0.,     8.,     0.,     9.,     8.,     2.,     8.,     0.,
    0.,     0.,     0.,     0.,     0.,     9.,     4.,     6.,     3.,     2.,     0.,
    0.,     0.,     0.,     0.,     1.,     1.,     9.,     1.,     5.,     5.,     3.,
    0.,     0.,     0.,     0.,     1.,     1.,     3.,     3.,     2.,     0.,     5.,
    0.,     0.,     0.,     0.,     0.,     0.,     0.,     0.,     2.,     4.,     6.,
    0.,     0.,     0.,     0.,     0.,     0.,     0.,     0.,     6.,     3.,     4.).finished();

  // Create factor graph
  const SharedDiagonal sig_4D = noiseModel::Isotropic::Sigma(4, 0.5);
  const SharedDiagonal sig_2D = noiseModel::Isotropic::Sigma(2, 0.5);
  GaussianFactorGraph factors = {
    std::make_shared<JacobianFactor>(KeyVector{3, 5, 7, 9, 11}, VerticalBlockMatrix(Dims{2, 2, 2, 2, 2, 1}, Ab.block(0, 0, 4, 11)), sig_4D),
    std::make_shared<JacobianFactor>(KeyVector{5, 7, 9, 11}, VerticalBlockMatrix(Dims{2, 2, 2, 2, 1}, Ab.block(4, 2, 4, 9)), sig_4D),
    std::make_shared<JacobianFactor>(KeyVector{7, 9, 11}, VerticalBlockMatrix(Dims{2, 2, 2, 1}, Ab.block(8, 4, 4, 7)), sig_4D),
    std::make_shared<JacobianFactor>(KeyVector{11}, VerticalBlockMatrix(Dims{2, 1}, Ab.block(12, 8, 2, 3)), sig_2D)};

  // extract the dense matrix for the graph
  Matrix actualDense = factors.augmentedJacobian();
  EXPECT(assert_equal(2.0 * Ab, actualDense));

  // Expected augmented matrix, both GaussianConditional (first 6 rows) and remaining factor (next 4 rows)
  Matrix R = 2.0 * (Matrix(10, 11) <<
    -12.1244,  -5.1962,  -5.2786,  -8.6603, -10.5573,  -5.9385, -11.3820,  -7.2581,  -8.7427, -13.4440,  -5.3611,
    0.,   4.6904,   5.0254,   5.5432,   5.5737,   3.0153,  -3.0153,  -3.5635,  -3.9290,  -2.7412,   2.1625,
    0.,       0., -13.8160,  -8.7166, -10.2245,  -8.8666,  -8.7632,  -5.2544,  -6.9192, -10.5537,  -9.3250,
    0.,       0.,       0.,   6.5033,  -1.1453,   1.3179,   2.5768,   5.5503,   3.6524,   1.3491,  -2.5676,
    0.,       0.,       0.,       0.,  -9.6242,  -2.1148,  -9.3509, -10.5846,  -3.5366,  -6.8561,  -3.2277,
    0.,       0.,       0.,       0.,       0.,   9.7887,   4.3551,   5.7572,   2.7876,   0.1611,   1.1769,
    0.,       0.,       0.,       0.,       0.,       0., -11.1139,  -0.6521,  -2.1943,  -7.5529,  -0.9081,
    0.,       0.,       0.,       0.,       0.,       0.,       0.,  -4.6479,  -1.9367,  -6.5170,  -3.7685,
    0.,       0.,       0.,       0.,       0.,       0.,       0.,       0.,   8.2503,   3.3757,   6.8476,
    0.,       0.,       0.,       0.,       0.,       0.,       0.,       0.,       0.,  -5.7095,  -0.0090).finished();

  GaussianConditional expectedFragment(KeyVector{3,5,7,9,11}, 3,
                                       VerticalBlockMatrix(Dims{2, 2, 2, 2, 2, 1}, R.topRows(6)),
                                       noiseModel::Unit::Create(6));

  // Eliminate (3 frontal variables, 6 scalar columns) using QR !!!!
  GaussianFactorGraph::EliminationResult actual = EliminateQR(factors, Ordering{3, 5, 7});
  const JacobianFactor &actualJF = dynamic_cast<const JacobianFactor&>(*actual.second);

  EXPECT(assert_equal(expectedFragment, *actual.first, 0.001));
  EXPECT_LONGS_EQUAL(size_t(2), actualJF.keys().size());
  EXPECT(assert_equal(Key(9), actualJF.keys()[0]));
  EXPECT(assert_equal(Key(11), actualJF.keys()[1]));
  EXPECT(assert_equal(Matrix(R.block(6, 6, 4, 2)), actualJF.getA(actualJF.begin()), 0.001));
  EXPECT(assert_equal(Matrix(R.block(6, 8, 4, 2)), actualJF.getA(actualJF.begin()+1), 0.001));
  EXPECT(assert_equal(Vector(R.col(10).segment(6, 4)), actualJF.getb(), 0.001));
  EXPECT(assert_equal(*noiseModel::Diagonal::Sigmas(Vector4::Ones()), *actualJF.get_model(), 0.001));
}

/* ************************************************************************* */
TEST ( JacobianFactor, constraint_eliminate1 )
{
  // construct a linear constraint
  Vector v(2); v(0)=1.2; v(1)=3.4;
  JacobianFactor lc(1, I_2x2, v, noiseModel::Constrained::All(2));

  // eliminate it
  pair<GaussianConditional::shared_ptr, JacobianFactor::shared_ptr>
    actual = lc.eliminate(Ordering{1});

  // verify linear factor is a null ptr
  EXPECT(actual.second->empty());

  // verify conditional Gaussian
  Vector sigmas = Vector2(0.0, 0.0);
  GaussianConditional expCG(1, v, I_2x2, noiseModel::Diagonal::Sigmas(sigmas));
  EXPECT(assert_equal(expCG, *actual.first));
}

/* ************************************************************************* */
TEST ( JacobianFactor, constraint_eliminate2 )
{
  // Construct a linear constraint
  // RHS
  Vector b(2); b(0)=3.0; b(1)=4.0;

  // A1 - invertible
  Matrix2 A1;  A1 << 2,4, 2,1;

  // A2 - not invertible
  Matrix2 A2;  A2 << 2,4, 2,4;

  JacobianFactor lc(1, A1, 2, A2, b, noiseModel::Constrained::All(2));

  // eliminate x and verify results
  pair<GaussianConditional::shared_ptr, JacobianFactor::shared_ptr>
    actual = lc.eliminate(Ordering{1});

  // LF should be empty
  // It's tricky to create Eigen matrices that are only zero along one dimension
  Matrix m(1,2);
  Matrix Aempty = m.topRows(0);
  Vector bempty = m.block(0,0,0,1);
  JacobianFactor expectedLF(2, Aempty, bempty, noiseModel::Constrained::All(0));
  EXPECT(assert_equal(expectedLF, *actual.second));

  // verify CG
  Matrix2 R; R << 1,2, 0,1;
  Matrix2 S; S << 1,2, 0,0;
  Vector d = Vector2(3.0, 0.6666);
  Vector sigmas = Vector2(0.0, 0.0);
  GaussianConditional expectedCG(1, d, R, 2, S, noiseModel::Diagonal::Sigmas(sigmas));
  EXPECT(assert_equal(expectedCG, *actual.first, 1e-4));
}

/* ************************************************************************* */
TEST(JacobianFactor, OverdeterminedEliminate) {
  Matrix Ab(9, 4);
  Ab << 0, 1, 0, 0,  //
      0, 0, 1, 0,    //
      Matrix74::Ones();

  // Call Gaussian version
  Vector9 sigmas = Vector9::Ones();
  SharedDiagonal diagonal = noiseModel::Diagonal::Sigmas(sigmas);

  JacobianFactor factor(0, Ab.leftCols(3), Ab.col(3), diagonal);
  GaussianFactorGraph::EliminationResult actual = factor.eliminate(Ordering{0});

  Matrix expectedRd(3, 4);
  expectedRd << -2.64575131, -2.64575131, -2.64575131, -2.64575131,  //
      0.0, -1, 0, 0,                                                 //
      0.0, 0.0, -1, 0;
  GaussianConditional expectedConditional(0, expectedRd.col(3), expectedRd.leftCols(3),
                                          noiseModel::Unit::Create(3));
  EXPECT(assert_equal(expectedConditional, *actual.first, 1e-4));
  EXPECT(actual.second->empty());
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
