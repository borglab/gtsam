/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCholesky.cpp
 * @author  Richard Roberts
 * @date    Nov 5, 2010
 */

#include <gtsam/base/debug.h>
#include <gtsam/base/cholesky.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST(cholesky, choleskyPartial) {

  // choleskyPartial should only use the upper triangle, so this represents a
  // symmetric matrix.
  Matrix ABC = Matrix_(7,7,
                      4.0375,   3.4584,   3.5735,   2.4815,   2.1471,   2.7400,   2.2063,
                          0.,   4.7267,   3.8423,   2.3624,   2.8091,   2.9579,   2.5914,
                          0.,       0.,   5.1600,   2.0797,   3.4690,   3.2419,   2.9992,
                          0.,       0.,       0.,   1.8786,   1.0535,   1.4250,   1.3347,
                          0.,       0.,       0.,       0.,   3.0788,   2.6283,   2.3791,
                          0.,       0.,       0.,       0.,       0.,   2.9227,   2.4056,
                          0.,       0.,       0.,       0.,       0.,       0.,   2.5776);

  // Do partial Cholesky on 3 frontal scalar variables
  Matrix RSL(ABC);
  choleskyPartial(RSL, 3);

  // See the function comment for choleskyPartial, this decomposition should hold.
  Matrix R1 = RSL.transpose();
  Matrix R2 = RSL;
  R1.block(3, 3, 4, 4).setIdentity();

  R2.block(3, 3, 4, 4) = R2.block(3, 3, 4, 4).selfadjointView<Eigen::Upper>();

  Matrix actual = R1 * R2;

  Matrix expected = ABC.selfadjointView<Eigen::Upper>();
  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
TEST(cholesky, BadScalingCholesky) {
  Matrix A = Matrix_(2,2,
      1e-40, 0.0,
      0.0, 1.0);

  Matrix R(A.transpose() * A);
  choleskyPartial(R, 2);

  double expectedSqrtCondition = 1e-40;
  double actualSqrtCondition = R(0,0) / R(1,1);

  DOUBLES_EQUAL(expectedSqrtCondition, actualSqrtCondition, 1e-41);
}

/* ************************************************************************* */
TEST(cholesky, BadScalingSVD) {
  Matrix A = Matrix_(2,2,
      1.0, 0.0,
      0.0, 1e-40);

  Matrix U, V;
  Vector S;
  gtsam::svd(A, U, S, V);

  double expectedCondition = 1e40;
  double actualCondition = S(0) / S(1);

  DOUBLES_EQUAL(expectedCondition, actualCondition, 1e30);
}

/* ************************************************************************* */
TEST(cholesky, underconstrained) {
	Matrix L(6,6); L <<
		1,	0,	0,	0,	0,	0,
		1.11177808157954,	1.06204809504665,	0.507342638873381,	1.34953401829486,	1,	0,
		0.155864888199928,	1.10933048588373,	0.501255576961674,	1,	0,	0,
		1.12108665967793,	1.01584408366945,	1,	0,	0,	0,
		0.776164062474843,	0.117617236580373,	-0.0236628691347294,	0.814118199972143,	0.694309975328922,	1,
		0.1197220685104,	1,	0,	0,	0,	0;
	Matrix D1(6,6); D1 <<
		0.814723686393179,	0,	0,	0,	0,	0,
		0,	0.811780089277421,	0,	0,	0,	0,
		0,	0,	1.82596950680844,	0,	0,	0,
		0,	0,	0,	0.240287537694585,	0,	0,
		0,	0,	0,	0,	1.34342584865901,	0,
		0,	0,	0,	0,	0,	1e-12;
	Matrix D2(6,6); D2 <<
		0.814723686393179,	0,	0,	0,	0,	0,
		0,	0.811780089277421,	0,	0,	0,	0,
		0,	0,	1.82596950680844,	0,	0,	0,
		0,	0,	0,	0.240287537694585,	0,	0,
		0,	0,	0,	0,	0,	0,
		0,	0,	0,	0,	0,	0;
	Matrix D3(6,6); D3 <<
		0.814723686393179,	0,	0,	0,	0,	0,
		0,	0.811780089277421,	0,	0,	0,	0,
		0,	0,	1.82596950680844,	0,	0,	0,
		0,	0,	0,	0.240287537694585,	0,	0,
		0,	0,	0,	0,	-0.5,	0,
		0,	0,	0,	0,	0,	-0.6;

	Matrix A1 = L * D1 * L.transpose();
	Matrix A2 = L * D2 * L.transpose();
	Matrix A3 = L * D3 * L.transpose();

	LONGS_EQUAL(long(false), long(choleskyPartial(A1, 6)));
	LONGS_EQUAL(long(false), long(choleskyPartial(A2, 6)));
	LONGS_EQUAL(long(false), long(choleskyPartial(A3, 6)));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
