/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRegularImplicitSchurFactor.cpp
 * @brief   unit test implicit jacobian factors
 * @author  Frank Dellaert
 * @date    Oct 20, 2013
 */

#include <gtsam/slam/JacobianFactorQ.h>
#include <gtsam/slam/JacobianFactorQR.h>
#include <gtsam/slam/RegularImplicitSchurFactor.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/Point2.h>

#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/base/timing.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/range/adaptor/map.hpp>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

// F
const Matrix26 F0 = Matrix26::Ones();
const Matrix26 F1 = 2 * Matrix26::Ones();
const Matrix26 F3 = 3 * Matrix26::Ones();
const vector<Matrix26, Eigen::aligned_allocator<Matrix26> > FBlocks {F0, F1, F3};
const KeyVector keys {0, 1, 3};
// RHS and sigmas
const Vector b = (Vector(6) << 1., 2., 3., 4., 5., 6.).finished();

//*************************************************************************************
TEST( regularImplicitSchurFactor, creation ) {
  // Matrix E = Matrix::Ones(6,3);
  Matrix E = Matrix::Zero(6, 3);
  E.block<2,2>(0, 0) = I_2x2;
  E.block<2,3>(2, 0) = 2 * Matrix::Ones(2, 3);
  Matrix3 P = (E.transpose() * E).inverse();
  RegularImplicitSchurFactor<CalibratedCamera> expected(keys, FBlocks, E, P, b);
  Matrix expectedP = expected.getPointCovariance();
  EXPECT(assert_equal(expectedP, P));
}

/* ************************************************************************* */
TEST( regularImplicitSchurFactor, addHessianMultiply ) {

  Matrix E = Matrix::Zero(6, 3);
  E.block<2,2>(0, 0) = I_2x2;
  E.block<2,3>(2, 0) = 2 * Matrix::Ones(2, 3);
  E.block<2,2>(4, 1) = I_2x2;
  Matrix3 P = (E.transpose() * E).inverse();

  double alpha = 0.5;
  VectorValues xvalues = map_list_of //
  (0, Vector::Constant(6, 2))//
  (1, Vector::Constant(6, 4))//
  (2, Vector::Constant(6, 0))// distractor
  (3, Vector::Constant(6, 8));

  VectorValues yExpected = map_list_of//
  (0, Vector::Constant(6, 27))//
  (1, Vector::Constant(6, -40))//
  (2, Vector::Constant(6, 0))// distractor
  (3, Vector::Constant(6, 279));

  // Create full F
  size_t M=4, m = 3, d = 6;
  Matrix F(2 * m, d * M);
  F << F0, Matrix::Zero(2, d * 3), Matrix::Zero(2, d), F1, Matrix::Zero(2, d*2), Matrix::Zero(2, d * 3), F3;

  // Calculate expected result F'*alpha*(I - E*P*E')*F*x
  KeyVector keys2{0,1,2,3};
  Vector x = xvalues.vector(keys2);
  Vector expected = Vector::Zero(24);
  RegularImplicitSchurFactor<CalibratedCamera>::multiplyHessianAdd(F, E, P, alpha, x, expected);
  EXPECT(assert_equal(expected, yExpected.vector(keys2), 1e-8));

  // Create ImplicitSchurFactor
  RegularImplicitSchurFactor<CalibratedCamera> implicitFactor(keys, FBlocks, E, P, b);

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
  JacobianFactorQ<6, 2> jfQ(keys, FBlocks, E, P, b, model);

  // error
  double expectedError = 11875.083333333334;
  {
    EXPECT_DOUBLES_EQUAL(expectedError,jfQ.error(xvalues),1e-7)
    EXPECT_DOUBLES_EQUAL(expectedError,implicitFactor.errorJF(xvalues),1e-7)
    EXPECT_DOUBLES_EQUAL(11903.500000000007,implicitFactor.error(xvalues),1e-7)
  }

  {
    VectorValues yActual = zero;
    jfQ.multiplyHessianAdd(alpha, xvalues, yActual);
    EXPECT(assert_equal(yExpected, yActual, 1e-8));
    jfQ.multiplyHessianAdd(alpha, xvalues, yActual);
    EXPECT(assert_equal(2 * yExpected, yActual, 1e-8));
    jfQ.multiplyHessianAdd(-1, xvalues, yActual);
    EXPECT(assert_equal(zero, yActual, 1e-8));
  }

  { // check hessian Diagonal
    VectorValues diagExpected = jfQ.hessianDiagonal();
    VectorValues diagActual = implicitFactor.hessianDiagonal();
    EXPECT(assert_equal(diagExpected, diagActual, 1e-8));
  }

  { // check hessian Block Diagonal
    map<Key,Matrix> BD = jfQ.hessianBlockDiagonal();
    map<Key,Matrix> actualBD = implicitFactor.hessianBlockDiagonal();
    LONGS_EQUAL(3,actualBD.size());
    EXPECT(assert_equal(BD[0],actualBD[0]));
    EXPECT(assert_equal(BD[1],actualBD[1]));
    EXPECT(assert_equal(BD[3],actualBD[3]));
  }

  { // Raw memory Version
    std::fill(y, y + 24, 0);// zero y !
    jfQ.multiplyHessianAdd(alpha, xdata, y);
    EXPECT(assert_equal(expected, XMap(y), 1e-8));
    jfQ.multiplyHessianAdd(alpha, xdata, y);
    EXPECT(assert_equal(Vector(2 * expected), XMap(y), 1e-8));
    jfQ.multiplyHessianAdd(-1, xdata, y);
    EXPECT(assert_equal(Vector(0 * expected), XMap(y), 1e-8));
  }

  VectorValues expectedVV;
  expectedVV.insert(0,-3.5*Vector::Ones(6));
  expectedVV.insert(1,10*Vector::Ones(6)/3);
  expectedVV.insert(3,-19.5*Vector::Ones(6));
  { // Check gradientAtZero
    VectorValues actual = implicitFactor.gradientAtZero();
    EXPECT(assert_equal(expectedVV, jfQ.gradientAtZero(), 1e-8));
    EXPECT(assert_equal(expectedVV, implicitFactor.gradientAtZero(), 1e-8));
  }

  // Create JacobianFactorQR
  JacobianFactorQR<6, 2> jfQR(keys, FBlocks, E, P, b, model);
    EXPECT_DOUBLES_EQUAL(expectedError, jfQR.error(xvalues),1e-7)
  EXPECT(assert_equal(expectedVV,  jfQR.gradientAtZero(), 1e-8));
  {
    const SharedDiagonal model;
    VectorValues yActual = zero;
    jfQR.multiplyHessianAdd(alpha, xvalues, yActual);
    EXPECT(assert_equal(yExpected, yActual, 1e-8));
    jfQR.multiplyHessianAdd(alpha, xvalues, yActual);
    EXPECT(assert_equal(2 * yExpected, yActual, 1e-8));
    jfQR.multiplyHessianAdd(-1, xvalues, yActual);
    EXPECT(assert_equal(zero, yActual, 1e-8));
  }

  { // Raw memory Version
    std::fill(y, y + 24, 0);// zero y !
    jfQR.multiplyHessianAdd(alpha, xdata, y);
    EXPECT(assert_equal(expected, XMap(y), 1e-8));
    jfQR.multiplyHessianAdd(alpha, xdata, y);
    EXPECT(assert_equal(Vector(2 * expected), XMap(y), 1e-8));
    jfQR.multiplyHessianAdd(-1, xdata, y);
    EXPECT(assert_equal(Vector(0 * expected), XMap(y), 1e-8));
  }
  delete [] y;
}

/* ************************************************************************* */
TEST(regularImplicitSchurFactor, hessianDiagonal)
{
  /* TESTED AGAINST MATLAB
   *  F = [Vector::Ones(2,6) zeros(2,6) zeros(2,6)
        zeros(2,6) 2*Vector::Ones(2,6) zeros(2,6)
        zeros(2,6) zeros(2,6) 3*Vector::Ones(2,6)]
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
  RegularImplicitSchurFactor<CalibratedCamera> factor(keys, FBlocks, E, P, b);

  // hessianDiagonal
  VectorValues expected;
  expected.insert(0, 1.195652*Vector::Ones(6));
  expected.insert(1, 4.782608*Vector::Ones(6));
  expected.insert(3, 7.043478*Vector::Ones(6));
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
  Matrix I2 = I_2x2;
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

  // augmentedInformation (test just checks diagonals)
  Matrix actualInfo = factor.augmentedInformation();
  EXPECT(assert_equal(actualBD[0],actualInfo.block<6,6>(0,0)));
  EXPECT(assert_equal(actualBD[1],actualInfo.block<6,6>(6,6)));
  EXPECT(assert_equal(actualBD[3],actualInfo.block<6,6>(12,12)));

  // information (test just checks diagonals)
  Matrix actualInfo2 = factor.information();
  EXPECT(assert_equal(actualBD[0],actualInfo2.block<6,6>(0,0)));
  EXPECT(assert_equal(actualBD[1],actualInfo2.block<6,6>(6,6)));
  EXPECT(assert_equal(actualBD[3],actualInfo2.block<6,6>(12,12)));
}

/* ************************************************************************* */
int main(void) {
  TestResult tr;
  int result = TestRegistry::runAllTests(tr);
  return result;
}
//*************************************************************************************
