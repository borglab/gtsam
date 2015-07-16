/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRegularJacobianFactor.cpp
 * @brief   unit test regular jacobian factors
 * @author  Sungtae An
 * @date    Nov 12, 2014
 */

#include <gtsam/linear/RegularJacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

static const size_t fixedDim = 3;
static const size_t nrKeys = 3;

// Keys are assumed to be from 0 to n
namespace {
  namespace simple {
    // Terms we'll use
    const vector<pair<Key, Matrix> > terms = list_of<pair<Key,Matrix> >
      (make_pair(0, Matrix3::Identity()))
      (make_pair(1, 2*Matrix3::Identity()))
      (make_pair(2, 3*Matrix3::Identity()));

    // RHS and sigmas
    const Vector b = (Vector(3) << 1., 2., 3.).finished();
    const SharedDiagonal noise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.5,0.5,0.5).finished());
  }

  namespace simple2 {
    // Terms
    const vector<pair<Key, Matrix> > terms2 = list_of<pair<Key,Matrix> >
      (make_pair(0, 2*Matrix3::Identity()))
      (make_pair(1, 4*Matrix3::Identity()))
      (make_pair(2, 6*Matrix3::Identity()));

    // RHS
    const Vector b2 = (Vector(3) << 2., 4., 6.).finished();
  }
}

/* ************************************************************************* */
// Convert from double* to VectorValues
VectorValues double2vv(const double* x,
    const  size_t nrKeys, const size_t dim) {
  // create map with dimensions
  std::map<gtsam::Key, size_t> dims;
  for (size_t i = 0; i < nrKeys; i++)
    dims.insert(make_pair(i, dim));

  size_t n = nrKeys*dim;
  Vector xVec(n);
  for (size_t i = 0; i < n; i++){
    xVec(i) = x[i];
  }
  return VectorValues(xVec, dims);
}

/* ************************************************************************* */
void vv2double(const VectorValues& vv, double* y,
    const  size_t nrKeys, const size_t dim) {
  // create map with dimensions
  std::map<gtsam::Key, size_t> dims;
  for (size_t i = 0; i < nrKeys; i++)
    dims.insert(make_pair(i, dim));

  Vector yvector = vv.vector(dims);
  size_t n = nrKeys*dim;
  for (size_t j = 0; j < n; j++)
    y[j] = yvector[j];
}

/* ************************************************************************* */
TEST(RegularJacobianFactor, constructorNway)
{
  using namespace simple;
  JacobianFactor factor(terms[0].first, terms[0].second,
        terms[1].first, terms[1].second, terms[2].first, terms[2].second, b, noise);
  RegularJacobianFactor<fixedDim> regularFactor(terms, b, noise);

  LONGS_EQUAL((long)terms[2].first, (long)regularFactor.keys().back());
  EXPECT(assert_equal(terms[2].second, regularFactor.getA(regularFactor.end() - 1)));
  EXPECT(assert_equal(b, factor.getb()));
  EXPECT(assert_equal(b, regularFactor.getb()));
  EXPECT(noise == factor.get_model());
  EXPECT(noise == regularFactor.get_model());
}

/* ************************************************************************* */
TEST(RegularJacobianFactor, hessianDiagonal)
{
  using namespace simple;
  JacobianFactor factor(terms[0].first, terms[0].second,
          terms[1].first, terms[1].second, terms[2].first, terms[2].second, b, noise);
  RegularJacobianFactor<fixedDim> regularFactor(terms, b, noise);

  // we compute hessian diagonal from the standard Jacobian
  VectorValues expectedHessianDiagonal = factor.hessianDiagonal();

  // we compare against the Raw memory access implementation of hessianDiagonal
  double actualValue[9]={0};
  regularFactor.hessianDiagonal(actualValue);
  VectorValues actualHessianDiagonalRaw = double2vv(actualValue,nrKeys,fixedDim);
  EXPECT(assert_equal(expectedHessianDiagonal, actualHessianDiagonalRaw));
}

/* ************************************************************************* */
TEST(RegularJacobian, gradientAtZero)
{
  using namespace simple;
  JacobianFactor factor(terms[0].first, terms[0].second,
          terms[1].first, terms[1].second, terms[2].first, terms[2].second, b, noise);
  RegularJacobianFactor<fixedDim> regularFactor(terms, b, noise);

  // we compute gradient at zero from the standard Jacobian
  VectorValues expectedGradientAtZero = factor.gradientAtZero();

  //EXPECT(assert_equal(expectedGradientAtZero, regularFactor.gradientAtZero()));

  // we compare against the Raw memory access implementation of gradientAtZero
  double actualValue[9]={0};
  regularFactor.gradientAtZero(actualValue);
  VectorValues actualGradientAtZeroRaw = double2vv(actualValue,nrKeys,fixedDim);
  EXPECT(assert_equal(expectedGradientAtZero, actualGradientAtZeroRaw));
}

/* ************************************************************************* */
TEST(RegularJacobian, gradientAtZero_multiFactors)
{
  using namespace simple;
  JacobianFactor factor(terms[0].first, terms[0].second,
          terms[1].first, terms[1].second, terms[2].first, terms[2].second, b, noise);
  RegularJacobianFactor<fixedDim> regularFactor(terms, b, noise);

  // we compute gradient at zero from the standard Jacobian
  VectorValues expectedGradientAtZero = factor.gradientAtZero();

  // we compare against the Raw memory access implementation of gradientAtZero
  double actualValue[9]={0};
  regularFactor.gradientAtZero(actualValue);
  VectorValues actualGradientAtZeroRaw = double2vv(actualValue,nrKeys,fixedDim);
  EXPECT(assert_equal(expectedGradientAtZero, actualGradientAtZeroRaw));

  // One more factor
  using namespace simple2;
  JacobianFactor factor2(terms2[0].first, terms2[0].second,
          terms2[1].first, terms2[1].second, terms2[2].first, terms2[2].second, b2, noise);
  RegularJacobianFactor<fixedDim> regularFactor2(terms2, b2, noise);

  // we accumulate computed gradient at zero from the standard Jacobian
  VectorValues expectedGradientAtZero2 = expectedGradientAtZero.add(factor2.gradientAtZero());

  // we compare against the Raw memory access implementation of gradientAtZero
  regularFactor2.gradientAtZero(actualValue);
  VectorValues actualGradientAtZeroRaw2 = double2vv(actualValue,nrKeys,fixedDim);
  EXPECT(assert_equal(expectedGradientAtZero2, actualGradientAtZeroRaw2));

}

/* ************************************************************************* */
TEST(RegularJacobian, multiplyHessianAdd)
{
  using namespace simple;
  JacobianFactor factor(terms[0].first, terms[0].second,
            terms[1].first, terms[1].second, terms[2].first, terms[2].second, b, noise);
  RegularJacobianFactor<fixedDim> regularFactor(terms, b, noise);

  // arbitrary vector X
  VectorValues X;
  X.insert(0, (Vector(3) << 10.,20.,30.).finished());
  X.insert(1, (Vector(3) << 10.,20.,30.).finished());
  X.insert(2, (Vector(3) << 10.,20.,30.).finished());

  // arbitrary vector Y
  VectorValues Y;
  Y.insert(0, (Vector(3) << 10.,10.,10.).finished());
  Y.insert(1, (Vector(3) << 20.,20.,20.).finished());
  Y.insert(2, (Vector(3) << 30.,30.,30.).finished());

  // multiplyHessianAdd Y += alpha*A'A*X
  double alpha = 2.0;
  VectorValues expectedMHA = Y;
  factor.multiplyHessianAdd(alpha, X, expectedMHA);

  // create data for raw memory access
  double XRaw[9];
  vv2double(X, XRaw, nrKeys, fixedDim);

  // test 1st version: multiplyHessianAdd(double alpha, const double* x, double* y)
  double actualMHARaw[9];
  vv2double(Y, actualMHARaw, nrKeys, fixedDim);
  regularFactor.multiplyHessianAdd(alpha, XRaw, actualMHARaw);
  VectorValues actualMHARawVV = double2vv(actualMHARaw,nrKeys,fixedDim);
  EXPECT(assert_equal(expectedMHA,actualMHARawVV));

  // test 2nd version: multiplyHessianAdd(double alpha, const double* x, double* y, std::vector<size_t> keys)
  double actualMHARaw2[9];
  vv2double(Y, actualMHARaw2, nrKeys, fixedDim);
  vector<size_t> dims;
  size_t accumulatedDim = 0;
  for (size_t i = 0; i < nrKeys+1; i++){
    dims.push_back(accumulatedDim);
    accumulatedDim += fixedDim;
  }
  regularFactor.multiplyHessianAdd(alpha, XRaw, actualMHARaw2, dims);
  VectorValues actualMHARawVV2 = double2vv(actualMHARaw2,nrKeys,fixedDim);
  EXPECT(assert_equal(expectedMHA,actualMHARawVV2));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
