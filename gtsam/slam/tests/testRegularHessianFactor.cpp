/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRegularHessianFactor.cpp
 * @author  Frank Dellaert
 * @date    March 4, 2014
 */

#include <gtsam/linear/VectorValues.h>
#include <CppUnitLite/TestHarness.h>

//#include <gtsam_unstable/slam/RegularHessianFactor.h>
#include <gtsam/slam/RegularHessianFactor.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/std/map.hpp>
#include <boost/assign/list_of.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

const double tol = 1e-5;

/* ************************************************************************* */
TEST(RegularHessianFactor, ConstructorNWay)
{
  Matrix G11 = (Matrix(2,2) << 111, 112, 113, 114).finished();
  Matrix G12 = (Matrix(2,2) << 121, 122, 123, 124).finished();
  Matrix G13 = (Matrix(2,2) << 131, 132, 133, 134).finished();

  Matrix G22 = (Matrix(2,2) << 221, 222, 222, 224).finished();
  Matrix G23 = (Matrix(2,2) << 231, 232, 233, 234).finished();

  Matrix G33 = (Matrix(2,2) << 331, 332, 332, 334).finished();

  Vector g1 = (Vector(2) << -7, -9).finished();
  Vector g2 = (Vector(2) << -9,  1).finished();
  Vector g3 = (Vector(2) <<  2,  3).finished();

  double f = 10;

  std::vector<Key> js;
  js.push_back(0); js.push_back(1); js.push_back(3);
  std::vector<Matrix> Gs;
  Gs.push_back(G11); Gs.push_back(G12); Gs.push_back(G13); Gs.push_back(G22); Gs.push_back(G23); Gs.push_back(G33);
  std::vector<Vector> gs;
  gs.push_back(g1); gs.push_back(g2); gs.push_back(g3);
  RegularHessianFactor<2> factor(js, Gs, gs, f);

  // multiplyHessianAdd:
  {
  // brute force
  Matrix AtA = factor.information();
  HessianFactor::const_iterator i1 = factor.begin();
  HessianFactor::const_iterator i2 = i1 + 1;
  Vector X(6); X << 1,2,3,4,5,6;
  Vector Y(6); Y << 2633, 2674, 4465, 4501, 5669, 5696;
  EXPECT(assert_equal(Y,AtA*X));

  VectorValues x = map_list_of<Key, Vector>
    (0, (Vector(2) << 1,2).finished())
    (1, (Vector(2) << 3,4).finished())
    (3, (Vector(2) << 5,6).finished());

  VectorValues expected;
  expected.insert(0, Y.segment<2>(0));
  expected.insert(1, Y.segment<2>(2));
  expected.insert(3, Y.segment<2>(4));

  // RAW ACCESS
  Vector expected_y(8); expected_y << 2633, 2674, 4465, 4501, 0, 0, 5669, 5696;
  Vector fast_y = gtsam::zero(8);
  double xvalues[8] = {1,2,3,4,0,0,5,6};
  factor.multiplyHessianAdd(1, xvalues, fast_y.data());
  EXPECT(assert_equal(expected_y, fast_y));

  // now, do it with non-zero y
  factor.multiplyHessianAdd(1, xvalues, fast_y.data());
  EXPECT(assert_equal(2*expected_y, fast_y));

  // check some expressions
  EXPECT(assert_equal(G12,factor.info(i1,i2).knownOffDiagonal()));
  EXPECT(assert_equal(G22,factor.info(i2,i2).selfadjointView()));
  EXPECT(assert_equal((Matrix)G12.transpose(),factor.info(i2,i1).knownOffDiagonal()));
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
