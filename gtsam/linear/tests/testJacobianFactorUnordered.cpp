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

#include <gtsam/linear/JacobianFactorUnordered.h>
#include <gtsam/linear/GaussianFactorGraphUnordered.h>
#include <gtsam/linear/GaussianConditionalUnordered.h>
#include <gtsam/linear/VectorValuesUnordered.h>

#include <boost/assign/list_of.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

namespace {
  namespace simple {
    // Terms we'll use
    const vector<pair<Key, Matrix> > terms = list_of<pair<Key,Matrix> >
      (make_pair(5, Matrix3::Identity()))
      (make_pair(10, 2*Matrix3::Identity()))
      (make_pair(15, 3*Matrix3::Identity()));

    // RHS and sigmas
    const Vector b = Vector_(3, 1., 2., 3.);
    const SharedDiagonal noise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.5, 0.5, 0.5));
  }
}

/* ************************************************************************* */
TEST(JacobianFactorUnordered, constructors_and_accessors)
{
  using namespace simple;

  // Test for using different numbers of terms
  {
    // b vector only constructor
    JacobianFactorUnordered expected(
      boost::make_iterator_range(terms.begin(), terms.begin()), b);
    JacobianFactorUnordered actual(b);
    EXPECT(assert_equal(expected, actual));
    EXPECT(assert_equal(b, expected.getb()));
    EXPECT(assert_equal(b, actual.getb()));
    EXPECT(!expected.get_model());
    EXPECT(!actual.get_model());
  }
  {
    // One term constructor
    JacobianFactorUnordered expected(
      boost::make_iterator_range(terms.begin(), terms.begin() + 1), b, noise);
    JacobianFactorUnordered actual(terms[0].first, terms[0].second, b, noise);
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
    JacobianFactorUnordered expected(
      boost::make_iterator_range(terms.begin(), terms.begin() + 2), b, noise);
    JacobianFactorUnordered actual(terms[0].first, terms[0].second,
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
    JacobianFactorUnordered expected(
      boost::make_iterator_range(terms.begin(), terms.begin() + 3), b, noise);
    JacobianFactorUnordered actual(terms[0].first, terms[0].second,
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
    // VerticalBlockMatrix constructor
    JacobianFactorUnordered expected(
      boost::make_iterator_range(terms.begin(), terms.begin() + 3), b, noise);
    VerticalBlockMatrix blockMatrix(list_of(3)(3)(3)(1), 3);
    blockMatrix(0) = terms[0].second;
    blockMatrix(1) = terms[1].second;
    blockMatrix(2) = terms[2].second;
    blockMatrix(3) = b;
    JacobianFactorUnordered actual(terms | boost::adaptors::map_keys, blockMatrix, noise);
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
//TEST(JabobianFactor, Hessian_conversion) {
//  HessianFactor hessian(0, (Matrix(4,4) <<
//        1.57,        2.695,         -1.1,        -2.35,
//       2.695,      11.3125,        -0.65,      -10.225,
//        -1.1,        -0.65,            1,          0.5,
//       -2.35,      -10.225,          0.5,         9.25).finished(),
//      (Vector(4) << -7.885, -28.5175, 2.75, 25.675).finished(),
//      73.1725);
//
//  JacobianFactor expected(0, (Matrix(2,4) <<
//      1.2530,   2.1508,   -0.8779,  -1.8755,
//           0,   2.5858,    0.4789,  -2.3943).finished(),
//      (Vector(2) << -6.2929, -5.7941).finished(),
//      noiseModel::Unit::Create(2));
//
//  JacobianFactor actual(hessian);
//
//  EXPECT(assert_equal(expected, actual, 1e-3));
//}

/* ************************************************************************* */
TEST( JacobianFactorUnordered, construct_from_graph)
{
  GaussianFactorGraphUnordered factors;

  double sigma1 = 0.1;
  Matrix A11 = Matrix::Identity(2,2);
  Vector b1(2); b1 << 2, -1;
  factors.add(JacobianFactorUnordered(10, A11, b1, noiseModel::Isotropic::Sigma(2, sigma1)));

  double sigma2 = 0.5;
  Matrix A21 = -10 * Matrix::Identity(2,2);
  Matrix A22 = 10 * Matrix::Identity(2,2);
  Vector b2(2); b2 << 4, -5;
  factors.add(JacobianFactorUnordered(10, A21, 8, A22, b2, noiseModel::Isotropic::Sigma(2, sigma2)));

  double sigma3 = 1.0;
  Matrix A32 = -10 * Matrix::Identity(2,2);
  Matrix A33 = 10 * Matrix::Identity(2,2);
  Vector b3(2); b3 << 4, -5;
  factors.add(JacobianFactorUnordered(8, A32, 12, A33, b3, noiseModel::Isotropic::Sigma(2, sigma3)));

  Matrix A1(6,2); A1 << A11, A21, Matrix::Zero(2,2);
  Matrix A2(6,2); A2 << Matrix::Zero(2,2), A22, A32;
  Matrix A3(6,2); A3 << Matrix::Zero(4,2), A33;
  Vector b(6); b << b1, b2, b3;
  Vector sigmas(6); sigmas << sigma1, sigma1, sigma2, sigma2, sigma3, sigma3;
  JacobianFactorUnordered expected(10, A1, 8, A2, 12, A3, b, noiseModel::Diagonal::Sigmas(sigmas));

  // The ordering here specifies the order in which the variables will appear in the combined factor
  JacobianFactorUnordered actual(factors, OrderingUnordered(list_of(10)(8)(12)));

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(JacobianFactorUnordered, error)
{
  JacobianFactorUnordered factor(simple::terms, simple::b, simple::noise);

  VectorValuesUnordered values;
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
TEST(JacobianFactorUnordered, matrices)
{
  JacobianFactorUnordered factor(simple::terms, simple::b, simple::noise);

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
  EXPECT(assert_equal(jacobianExpected, factor.jacobian(false).first));
  EXPECT(assert_equal(rhsExpected, factor.jacobian(false).second));
  EXPECT(assert_equal(augmentedJacobianExpected, factor.augmentedJacobian(false)));
}

/* ************************************************************************* */
TEST(JacobianFactorUnordered, operators )
{
  SharedDiagonal  sigma0_1 = noiseModel::Isotropic::Sigma(2,0.1);

  Matrix I = eye(2);
  Vector b = Vector_(2,0.2,-0.1);
  JacobianFactorUnordered lf(1, -I, 2, I, b, sigma0_1);

  VectorValuesUnordered c;
  c.insert(1, Vector_(2,10.,20.));
  c.insert(2, Vector_(2,30.,60.));

  // test A*x
  Vector expectedE = Vector_(2,200.,400.);
  Vector actualE = lf * c;
  EXPECT(assert_equal(expectedE, actualE));

  // test A^e
  VectorValuesUnordered expectedX;
  expectedX.insert(1, Vector_(2,-2000.,-4000.));
  expectedX.insert(2, Vector_(2, 2000., 4000.));
  VectorValuesUnordered actualX = VectorValuesUnordered::Zero(expectedX);
  lf.transposeMultiplyAdd(1.0, actualE, actualX);
  EXPECT(assert_equal(expectedX, actualX));
}

/* ************************************************************************* */
TEST(JacobianFactorUnordered, default_error )
{
  JacobianFactorUnordered f;
  double actual = f.error(VectorValuesUnordered());
  DOUBLES_EQUAL(0.0, actual, 1e-15);
}

//* ************************************************************************* */
TEST(JacobianFactorUnordered, empty )
{
  // create an empty factor
  JacobianFactorUnordered f;
  EXPECT(f.empty());
}

/* ************************************************************************* */
TEST(JacobianFactorUnordered, eliminate2 )
{
  // sigmas
  double sigma1 = 0.2;
  double sigma2 = 0.1;
  Vector sigmas = Vector_(4, sigma1, sigma1, sigma2, sigma2);

  // the combined linear factor
  Matrix Ax2 = Matrix_(4,2,
    // x2
    -1., 0.,
    +0.,-1.,
    1., 0.,
    +0.,1.
    );

  Matrix Al1x1 = Matrix_(4,4,
    // l1   x1
    1., 0., 0.00,  0., // f4
    0., 1., 0.00,  0., // f4
    0., 0., -1.,  0., // f2
    0., 0., 0.00,-1.  // f2
    );

  // the RHS
  Vector b2(4);
  b2(0) = -0.2;
  b2(1) =  0.3;
  b2(2) =  0.2;
  b2(3) = -0.1;

  vector<pair<Index, Matrix> > meas;
  meas.push_back(make_pair(2, Ax2));
  meas.push_back(make_pair(11, Al1x1));
  JacobianFactorUnordered combined(meas, b2, noiseModel::Diagonal::Sigmas(sigmas));

  // eliminate the combined factor
  pair<GaussianConditionalUnordered::shared_ptr, JacobianFactorUnordered::shared_ptr>
    actual = combined.eliminate(OrderingUnordered(list_of(2)));

  // create expected Conditional Gaussian
  double oldSigma = 0.0894427; // from when R was made unit
  Matrix R11 = Matrix_(2,2,
    1.00,  0.00,
    0.00,  1.00
    )/oldSigma;
  Matrix S12 = Matrix_(2,4,
    -0.20, 0.00,-0.80, 0.00,
    +0.00,-0.20,+0.00,-0.80
    )/oldSigma;
  Vector d = Vector_(2,0.2,-0.14)/oldSigma;
  GaussianConditionalUnordered expectedCG(2, d, R11, 11, S12);

  EXPECT(assert_equal(expectedCG, *actual.first, 1e-4));

  // the expected linear factor
  double sigma = 0.2236;
  Matrix Bl1x1 = Matrix_(2,4,
    // l1          x1
    1.00, 0.00, -1.00,  0.00,
    0.00, 1.00, +0.00, -1.00
    )/sigma;
  Vector b1 = Vector_(2, 0.0, 0.894427);
  JacobianFactorUnordered expectedLF(11, Bl1x1, b1);
  EXPECT(assert_equal(expectedLF, *actual.second,1e-3));
}

/* ************************************************************************* */
TEST ( JacobianFactorUnordered, constraint_eliminate1 )
{
  // construct a linear constraint
  Vector v(2); v(0)=1.2; v(1)=3.4;
  JacobianFactorUnordered lc(1, eye(2), v, noiseModel::Constrained::All(2));

  // eliminate it
  pair<GaussianConditionalUnordered::shared_ptr, JacobianFactorUnordered::shared_ptr>
    actual = lc.eliminate(list_of(1));

  // verify linear factor
  EXPECT(actual.second->size() == 0);

  // verify conditional Gaussian
  Vector sigmas = Vector_(2, 0.0, 0.0);
  GaussianConditionalUnordered expCG(1, v, eye(2), noiseModel::Diagonal::Sigmas(sigmas));
  EXPECT(assert_equal(expCG, *actual.first));
}

/* ************************************************************************* */
TEST ( JacobianFactorUnordered, constraint_eliminate2 )
{
  // Construct a linear constraint
  // RHS
  Vector b(2); b(0)=3.0; b(1)=4.0;

  // A1 - invertible
  Matrix A1(2,2);
  A1(0,0) = 1.0 ; A1(0,1) = 2.0;
  A1(1,0) = 2.0 ; A1(1,1) = 1.0;

  // A2 - not invertible
  Matrix A2(2,2);
  A2(0,0) = 1.0 ; A2(0,1) = 2.0;
  A2(1,0) = 2.0 ; A2(1,1) = 4.0;

  JacobianFactorUnordered lc(1, A1, 2, A2, b, noiseModel::Constrained::All(2));

  // eliminate x and verify results
  pair<GaussianConditionalUnordered::shared_ptr, JacobianFactorUnordered::shared_ptr>
    actual = lc.eliminate(list_of(1));

  // LF should be empty
  // It's tricky to create Eigen matrices that are only zero along one dimension
  Matrix m(1,2);
  Matrix Aempty = m.topRows(0);
  Vector bempty = m.block(0,0,0,1);
  JacobianFactorUnordered expectedLF(2, Aempty, bempty, noiseModel::Constrained::All(0));
  EXPECT(assert_equal(expectedLF, *actual.second));

  // verify CG
  Matrix R = Matrix_(2, 2,
      1.0,    2.0,
      0.0,    1.0);
  Matrix S = Matrix_(2,2,
      1.0,    2.0,
      0.0,    0.0);
  Vector d = Vector_(2, 3.0, 0.6666);
  Vector sigmas = Vector_(2, 0.0, 0.0);
  GaussianConditionalUnordered expectedCG(1, d, R, 2, S, noiseModel::Diagonal::Sigmas(sigmas));
  EXPECT(assert_equal(expectedCG, *actual.first, 1e-4));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
