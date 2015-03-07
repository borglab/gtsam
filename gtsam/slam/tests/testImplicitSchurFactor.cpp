/**
 * @file    testImplicitSchurFactor.cpp
 * @brief   unit test implicit jacobian factors
 * @author  Frank Dellaert
 * @date    Oct 20, 2013
 */

//#include <gtsam_unstable/slam/ImplicitSchurFactor.h>
#include <gtsam/slam/ImplicitSchurFactor.h>
//#include <gtsam_unstable/slam/JacobianFactorQ.h>
#include <gtsam/slam/JacobianFactorQ.h>
//#include "gtsam_unstable/slam/JacobianFactorQR.h"
#include "gtsam/slam/JacobianFactorQR.h"

#include <gtsam/base/timing.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/GaussianFactor.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/range/adaptor/map.hpp>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

// F
typedef Eigen::Matrix<double, 2, 6> Matrix26;
const Matrix26 F0 = Matrix26::Ones();
const Matrix26 F1 = 2 * Matrix26::Ones();
const Matrix26 F3 = 3 * Matrix26::Ones();
const vector<pair<Key, Matrix26> > Fblocks = list_of<pair<Key, Matrix> > //
    (make_pair(0, F0))(make_pair(1, F1))(make_pair(3, F3));
// RHS and sigmas
const Vector b = (Vector(6) << 1., 2., 3., 4., 5., 6.);

//*************************************************************************************
TEST( implicitSchurFactor, creation ) {
  // Matrix E = Matrix::Ones(6,3);
  Matrix E = zeros(6, 3);
  E.block<2,2>(0, 0) = eye(2);
  E.block<2,3>(2, 0) = 2 * ones(2, 3);
  Matrix3 P = (E.transpose() * E).inverse();
  ImplicitSchurFactor<6> expected(Fblocks, E, P, b);
  Matrix expectedP = expected.getPointCovariance();
  EXPECT(assert_equal(expectedP, P));
}

/* ************************************************************************* */
TEST( implicitSchurFactor, addHessianMultiply ) {

  Matrix E = zeros(6, 3);
  E.block<2,2>(0, 0) = eye(2);
  E.block<2,3>(2, 0) = 2 * ones(2, 3);
  E.block<2,2>(4, 1) = eye(2);
  Matrix3 P = (E.transpose() * E).inverse();

  double alpha = 0.5;
  VectorValues xvalues = map_list_of //
  (0, gtsam::repeat(6, 2))//
  (1, gtsam::repeat(6, 4))//
  (2, gtsam::repeat(6, 0))// distractor
  (3, gtsam::repeat(6, 8));

  VectorValues yExpected = map_list_of//
  (0, gtsam::repeat(6, 27))//
  (1, gtsam::repeat(6, -40))//
  (2, gtsam::repeat(6, 0))// distractor
  (3, gtsam::repeat(6, 279));

  // Create full F
  size_t M=4, m = 3, d = 6;
  Matrix F(2 * m, d * M);
  F << F0, zeros(2, d * 3), zeros(2, d), F1, zeros(2, d*2), zeros(2, d * 3), F3;

  // Calculate expected result F'*alpha*(I - E*P*E')*F*x
  FastVector<Key> keys;
  keys += 0,1,2,3;
  Vector x = xvalues.vector(keys);
  Vector expected = zero(24);
  ImplicitSchurFactor<6>::multiplyHessianAdd(F, E, P, alpha, x, expected);
  EXPECT(assert_equal(expected, yExpected.vector(keys), 1e-8));

  // Create ImplicitSchurFactor
  ImplicitSchurFactor<6> implicitFactor(Fblocks, E, P, b);

  VectorValues zero = 0 * yExpected;// quick way to get zero w right structure
  { // First Version
    VectorValues yActual = zero;
    implicitFactor.multiplyHessianAdd(alpha, xvalues, yActual);
    EXPECT(assert_equal(yExpected, yActual, 1e-8));
    implicitFactor.multiplyHessianAdd(alpha, xvalues, yActual);
    EXPECT(assert_equal(2 * yExpected, yActual, 1e-8));
    implicitFactor.multiplyHessianAdd(-1, xvalues, yActual);
    EXPECT(assert_equal(zero, yActual, 1e-8));
  }

  typedef Eigen::Matrix<double, 24, 1> DeltaX;
  typedef Eigen::Map<DeltaX> XMap;
  double* y = new double[24];
  double* xdata = x.data();

  { // Raw memory Version
    std::fill(y, y + 24, 0);// zero y !
    implicitFactor.multiplyHessianAdd(alpha, xdata, y);
    EXPECT(assert_equal(expected, XMap(y), 1e-8));
    implicitFactor.multiplyHessianAdd(alpha, xdata, y);
    EXPECT(assert_equal(Vector(2 * expected), XMap(y), 1e-8));
    implicitFactor.multiplyHessianAdd(-1, xdata, y);
    EXPECT(assert_equal(Vector(0 * expected), XMap(y), 1e-8));
  }

  // Create JacobianFactor with same error
  const SharedDiagonal model;
  JacobianFactorQ<6> jf(Fblocks, E, P, b, model);

  { // error
    double expectedError = jf.error(xvalues);
    double actualError = implicitFactor.errorJF(xvalues);
    DOUBLES_EQUAL(expectedError,actualError,1e-7)
  }

  { // JacobianFactor with same error
    VectorValues yActual = zero;
    jf.multiplyHessianAdd(alpha, xvalues, yActual);
    EXPECT(assert_equal(yExpected, yActual, 1e-8));
    jf.multiplyHessianAdd(alpha, xvalues, yActual);
    EXPECT(assert_equal(2 * yExpected, yActual, 1e-8));
    jf.multiplyHessianAdd(-1, xvalues, yActual);
    EXPECT(assert_equal(zero, yActual, 1e-8));
  }

  { // check hessian Diagonal
    VectorValues diagExpected = jf.hessianDiagonal();
    VectorValues diagActual = implicitFactor.hessianDiagonal();
    EXPECT(assert_equal(diagExpected, diagActual, 1e-8));
  }

  { // check hessian Block Diagonal
    map<Key,Matrix> BD = jf.hessianBlockDiagonal();
    map<Key,Matrix> actualBD = implicitFactor.hessianBlockDiagonal();
    LONGS_EQUAL(3,actualBD.size());
    EXPECT(assert_equal(BD[0],actualBD[0]));
    EXPECT(assert_equal(BD[1],actualBD[1]));
    EXPECT(assert_equal(BD[3],actualBD[3]));
  }

  { // Raw memory Version
    std::fill(y, y + 24, 0);// zero y !
    jf.multiplyHessianAdd(alpha, xdata, y);
    EXPECT(assert_equal(expected, XMap(y), 1e-8));
    jf.multiplyHessianAdd(alpha, xdata, y);
    EXPECT(assert_equal(Vector(2 * expected), XMap(y), 1e-8));
    jf.multiplyHessianAdd(-1, xdata, y);
    EXPECT(assert_equal(Vector(0 * expected), XMap(y), 1e-8));
  }

  { // Check gradientAtZero
    VectorValues expected = jf.gradientAtZero();
    VectorValues actual = implicitFactor.gradientAtZero();
    EXPECT(assert_equal(expected, actual, 1e-8));
  }

  // Create JacobianFactorQR
  JacobianFactorQR<6> jfq(Fblocks, E, P, b, model);
  {
    const SharedDiagonal model;
    VectorValues yActual = zero;
    jfq.multiplyHessianAdd(alpha, xvalues, yActual);
    EXPECT(assert_equal(yExpected, yActual, 1e-8));
    jfq.multiplyHessianAdd(alpha, xvalues, yActual);
    EXPECT(assert_equal(2 * yExpected, yActual, 1e-8));
    jfq.multiplyHessianAdd(-1, xvalues, yActual);
    EXPECT(assert_equal(zero, yActual, 1e-8));
  }

  { // Raw memory Version
    std::fill(y, y + 24, 0);// zero y !
    jfq.multiplyHessianAdd(alpha, xdata, y);
    EXPECT(assert_equal(expected, XMap(y), 1e-8));
    jfq.multiplyHessianAdd(alpha, xdata, y);
    EXPECT(assert_equal(Vector(2 * expected), XMap(y), 1e-8));
    jfq.multiplyHessianAdd(-1, xdata, y);
    EXPECT(assert_equal(Vector(0 * expected), XMap(y), 1e-8));
  }
  delete [] y;
}

/* ************************************************************************* */
TEST(implicitSchurFactor, hessianDiagonal)
{
  /* TESTED AGAINST MATLAB
   *  F = [ones(2,6) zeros(2,6) zeros(2,6)
        zeros(2,6) 2*ones(2,6) zeros(2,6)
        zeros(2,6) zeros(2,6) 3*ones(2,6)]
      E = [[1:6] [1:6] [0.5 1:5]];
      E = reshape(E',3,6)'
      P = inv(E' * E)
      H = F' * (eye(6) - E * P * E') * F
      diag(H)
   */
  Matrix E(6,3);
  E.block<2,3>(0, 0) << 1,2,3,4,5,6;
  E.block<2,3>(2, 0) << 1,2,3,4,5,6;
  E.block<2,3>(4, 0) << 0.5,1,2,3,4,5;
  Matrix3 P = (E.transpose() * E).inverse();
  ImplicitSchurFactor<6> factor(Fblocks, E, P, b);

  // hessianDiagonal
  VectorValues expected;
  expected.insert(0, 1.195652*ones(6));
  expected.insert(1, 4.782608*ones(6));
  expected.insert(3, 7.043478*ones(6));
  EXPECT(assert_equal(expected, factor.hessianDiagonal(),1e-5));

  // hessianBlockDiagonal
  map<Key,Matrix> actualBD = factor.hessianBlockDiagonal();
  LONGS_EQUAL(3,actualBD.size());
  Matrix FtE0 = F0.transpose() * E.block<2,3>(0, 0);
  Matrix FtE1 = F1.transpose() * E.block<2,3>(2, 0);
  Matrix FtE3 = F3.transpose() * E.block<2,3>(4, 0);

  // variant one
  EXPECT(assert_equal(F0.transpose()*F0-FtE0*P*FtE0.transpose(),actualBD[0]));
  EXPECT(assert_equal(F1.transpose()*F1-FtE1*P*FtE1.transpose(),actualBD[1]));
  EXPECT(assert_equal(F3.transpose()*F3-FtE3*P*FtE3.transpose(),actualBD[3]));

  // variant two
  Matrix I2 = eye(2);
  Matrix E0 = E.block<2,3>(0, 0);
  Matrix F0t = F0.transpose();
  EXPECT(assert_equal(F0t*F0-F0t*E0*P*E0.transpose()*F0,actualBD[0]));
  EXPECT(assert_equal(F0t*(F0-E0*P*E0.transpose()*F0),actualBD[0]));

  Matrix M1 = F0t*(F0-E0*P*E0.transpose()*F0);
  Matrix M2 = F0t*F0-F0t*E0*P*E0.transpose()*F0;

  EXPECT(assert_equal(  M1 , actualBD[0] ));
  EXPECT(assert_equal(  M1 , M2 ));

  Matrix M1b = F0t*(E0*P*E0.transpose()*F0);
  Matrix M2b = F0t*E0*P*E0.transpose()*F0;
  EXPECT(assert_equal(  M1b , M2b ));

  EXPECT(assert_equal(F0t*(I2-E0*P*E0.transpose())*F0,actualBD[0]));
  EXPECT(assert_equal(F1.transpose()*F1-FtE1*P*FtE1.transpose(),actualBD[1]));
  EXPECT(assert_equal(F3.transpose()*F3-FtE3*P*FtE3.transpose(),actualBD[3]));
}

/* ************************************************************************* */
int main(void) {
  TestResult tr;
  int result = TestRegistry::runAllTests(tr);
  return result;
}
//*************************************************************************************
