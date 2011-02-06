/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCholesky.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Nov 5, 2010
 */

#include <gtsam/base/debug.h>
#include <gtsam/base/cholesky.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/symmetric.hpp>

using namespace gtsam;
namespace ublas = boost::numeric::ublas;

/* ************************************************************************* */
TEST(cholesky, cholesky_inplace) {
  Matrix I = Matrix_(5,5,
         1.739214152118470,   0.714164147286383,   1.893437990040579,   1.469485110232169,   1.549910412077052,
         0.714164147286383,   0.587767128354794,   0.738548568260523,   0.588775778746414,   0.645015863153235,
         1.893437990040579,   0.738548568260523,   2.165530165155413,   1.631203698494913,   1.617901992621506,
         1.469485110232169,   0.588775778746414,   1.631203698494913,   1.591363675348797,   1.584779118350080,
         1.549910412077052,   0.645015863153235,   1.617901992621506,   1.584779118350080,   1.928887183904716);

  Matrix expected = Matrix_(5,5,
      1.318792687316119,   0.541528743793524,   1.435735888021887,   1.114265437142152,   1.175249473995251,
                    0.0,   0.542691208699940,  -0.071760299365346,  -0.026960052875681,   0.015818372803848,
                    0.0,                 0.0,   0.314711112667495,   0.093667363355578,  -0.217058504307151,
                    0.0,                 0.0,                 0.0,   0.583331630832890,   0.507424929100112,
                    0.0,                 0.0,                 0.0,                 0.0,   0.492779041656733);

  MatrixColMajor actualColMaj = I;
  cholesky_inplace(actualColMaj);
  Matrix actual = ublas::triangular_adaptor<MatrixColMajor, ublas::upper>(actualColMaj);
  CHECK(assert_equal(expected, actual, 1e-7));
}

/* ************************************************************************* */
TEST(cholesky, choleskyFactorUnderdetermined) {

  Matrix Ab = Matrix_(3,7,
      0.0357,    0.6787,    0.3922,    0.7060,    0.0462,    0.6948,    0.0344,
      0.8491,    0.7577,    0.6555,    0.0318,    0.0971,    0.3171,    0.4387,
      0.9340,    0.7431,    0.1712,    0.2769,    0.8235,    0.9502,    0.3816);

  Matrix expected = Matrix_(3,7,
      1.2628,    1.0784,    0.5785,    0.2462,    0.6757,    0.9357,    0.5782,
         0.0,    0.6513,    0.4089,    0.6811,   -0.0180,    0.6280,    0.0244,
         0.0,       0.0,    0.3332,   -0.2273,   -0.4825,   -0.4652,    0.0660);

  MatrixColMajor actualColmaj = Ab;
  choleskyFactorUnderdetermined(actualColmaj, 3);
  Matrix actual = ublas::triangular_adaptor<MatrixColMajor, ublas::upper>(actualColmaj);

  CHECK(assert_equal(expected, actual, 1e-3));
}

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
  MatrixColMajor RSL(ABC);
  choleskyPartial(RSL, 3);

  // See the function comment for choleskyPartial, this decomposition should hold.
  Matrix R1(ublas::trans(RSL));
  Matrix R2(RSL);
  ublas::project(R1, ublas::range(3,7), ublas::range(3,7)) = eye(4,4);
  ublas::project(R2, ublas::range(3,7), ublas::range(3,7)) =
      ublas::symmetric_adaptor<const ublas::matrix_range<Matrix>,ublas::upper>(
          ublas::project(R2, ublas::range(3,7), ublas::range(3,7)));
  Matrix actual(R1 * R2);

  EXPECT(assert_equal(ublas::symmetric_adaptor<Matrix,ublas::upper>(ABC), actual, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
