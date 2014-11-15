/**
 * @file    testRegularJacobianFactor.cpp
 * @brief   unit test regular jacobian factors
 * @author  Sungtae An
 * @date    Nov 12, 2014
 */

#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

#include <gtsam/slam/RegularJacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>

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
    const Vector b = (Vector(3) << 1., 2., 3.);
    const SharedDiagonal noise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.5,0.5,0.5));
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

  // we compute hessian diagonal from the regular Jacobian, but still using the
  // implementation of hessianDiagonal in the base class
  //VectorValues actualHessianDiagonal = regularFactor.hessianDiagonal();

  //EXPECT(assert_equal(expectedHessianDiagonal,actualHessianDiagonal));

  // we compare against the Raw memory access implementation of hessianDiagonal
  double actualValue[9];
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

  // we compute gradient at zero from the regular Jacobian, but still using the
  // implementation of gradientAtZero in the base class
  VectorValues actualGradientAtZero = regularFactor.gradientAtZero();

  EXPECT(assert_equal(expectedGradientAtZero, regularFactor.gradientAtZero()));

  // we compare against the Raw memory access implementation of gradientAtZero
  double actualValue[9];
  regularFactor.gradientAtZero(actualValue);
  VectorValues actualGradientAtZeroRaw = double2vv(actualValue,nrKeys,fixedDim);
  EXPECT(assert_equal(expectedGradientAtZero, actualGradientAtZeroRaw));

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
  X.insert(0, (Vector(3) << 10.,20.,30.));
  X.insert(1, (Vector(3) << 10.,20.,30.));
  X.insert(2, (Vector(3) << 10.,20.,30.));

  // arbitrary vector Y
  VectorValues Y;
  Y.insert(0, (Vector(3) << 10.,10.,10.));
  Y.insert(1, (Vector(3) << 20.,20.,20.));
  Y.insert(2, (Vector(3) << 30.,30.,30.));

  // multiplyHessianAdd Y += alpha*A'A*X
  double alpha = 2.0;
  VectorValues expectedMHA = Y;
  factor.multiplyHessianAdd(alpha, X, expectedMHA);

  VectorValues actualMHA = Y;
  regularFactor.multiplyHessianAdd(alpha, X, actualMHA);

  EXPECT(assert_equal(expectedMHA, actualMHA));

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
