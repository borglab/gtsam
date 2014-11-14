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

namespace {
  namespace simple {
    // Terms we'll use
    const vector<pair<Key, Matrix> > terms = list_of<pair<Key,Matrix> >
      (make_pair(5, Matrix3::Identity()))
      (make_pair(10, 2*Matrix3::Identity()))
      (make_pair(15, 3*Matrix3::Identity()));

    // RHS and sigmas
    const Vector b = (Vector(3) << 1., 2., 3.);
    const SharedDiagonal noise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.5, 0.5, 0.5));
  }
}

/* ************************************************************************* */
TEST(RegularJacobianFactor, constructorNway)
{
  using namespace simple;
  JacobianFactor expected(terms[0].first, terms[0].second,
        terms[1].first, terms[1].second, terms[2].first, terms[2].second, b, noise);
  RegularJacobianFactor<3> actual(terms, b, noise);

  LONGS_EQUAL((long)terms[2].first, (long)actual.keys().back());
  EXPECT(assert_equal(terms[2].second, actual.getA(actual.end() - 1)));
  EXPECT(assert_equal(b, expected.getb()));
  EXPECT(assert_equal(b, actual.getb()));
  EXPECT(noise == expected.get_model());
  EXPECT(noise == actual.get_model());
}

TEST(RegularJacobianFactor, hessianDiagonal)
{
  using namespace simple;
  JacobianFactor expected(terms[0].first, terms[0].second,
          terms[1].first, terms[1].second, terms[2].first, terms[2].second, b, noise);
  RegularJacobianFactor<3> actual(terms, b, noise);

  EXPECT(assert_equal(expected.hessianDiagonal(),actual.hessianDiagonal()));
  expected.hessianDiagonal().print();
  actual.hessianDiagonal().print();
  double actualValue[9];
  actual.hessianDiagonal(actualValue);
  for(int i=0; i<9; ++i)
    std::cout << actualValue[i] << std::endl;

  // Why unwhitened?
}

TEST(RegularJacobian, gradientAtZero)
{
  using namespace simple;
  JacobianFactor expected(terms[0].first, terms[0].second,
          terms[1].first, terms[1].second, terms[2].first, terms[2].second, b, noise);
  RegularJacobianFactor<3> actual(terms, b, noise);
  EXPECT(assert_equal(expected.gradientAtZero(),actual.gradientAtZero()));

  // raw memory access is not available now
}

TEST(RegularJacobian, multiplyHessianAdd)
{
  using namespace simple;
  RegularJacobianFactor<3> factor(terms, b, noise);

  VectorValues X;
  X.insert(5, (Vector(3) << 10.,20.,30.));
  X.insert(10, (Vector(3) << 10.,20.,30.));
  X.insert(15, (Vector(3) << 10.,20.,30.));

  VectorValues Y;
  Y.insert(5, (Vector(3) << 10.,10.,10.));
  Y.insert(10, (Vector(3) << 20.,20.,20.));
  Y.insert(15, (Vector(3) << 30.,30.,30.));

  // muultiplyHessianAdd Y += alpha*A'A*X
  factor.multiplyHessianAdd(2.0, X, Y);

  VectorValues expectedValues;
  expectedValues.insert(5, (Vector(3) << 490.,970.,1450.));
  expectedValues.insert(10, (Vector(3) << 980.,1940.,2900.));
  expectedValues.insert(15, (Vector(3) << 1470.,2910.,4350.));

  EXPECT(assert_equal(expectedValues,Y));

  //double dataX[9] = {10., 20., 30., 10., 20., 30., 10., 20., 30.};
  //double dataY[9] = {10., 10., 10., 20., 20., 20., 30., 30., 30.};

  std::cout << "size: " << factor.size() << std::endl;
  double dataX[9] = {0.,};
  double dataY[9] = {0.,};

  std::vector<Key> ks;
  ks.push_back(5);ks.push_back(10);ks.push_back(15);

  // Raw memory access version
  factor.multiplyHessianAdd(2.0, dataX, dataY, ks);

}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
