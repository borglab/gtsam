/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeCholesky.cpp
 * @brief   time Cholesky factorization
 * @author  Frank Dellaert
 * @date    March 4, 2016
 */

#include <gtsam/base/cholesky.h>

#include <time.h>
#include <iostream>
#include <iomanip>      // std::setprecision

using namespace std;
using namespace gtsam;

//#define TERNARY

int main() {

  Matrix top = (Matrix(7,7) <<
                      4.0375,   3.4584,   3.5735,   2.4815,   2.1471,   2.7400,   2.2063,
                          0.,   4.7267,   3.8423,   2.3624,   2.8091,   2.9579,   2.5914,
                          0.,       0.,   5.1600,   2.0797,   3.4690,   3.2419,   2.9992,
                          0.,       0.,       0.,   1.8786,   1.0535,   1.4250,   1.3347,
                          0.,       0.,       0.,       0.,   3.0788,   2.6283,   2.3791,
                          0.,       0.,       0.,       0.,       0.,   2.9227,   2.4056,
                          0.,       0.,       0.,       0.,       0.,       0.,   2.5776).finished();

  Matrix ABC(100,100);
  ABC.topLeftCorner<7,7>() = top;
  cout << setprecision(3);

  size_t n = 100000;
  for (size_t nFrontal = 1; nFrontal <= 7; nFrontal++) {
    auto timeLog = clock();
    for (size_t i = 0; i < n; i++) {
      Matrix RSL(ABC);
      choleskyPartial(RSL, nFrontal);
    }
    auto timeLog2 = clock();
    auto seconds = (double)(timeLog2 - timeLog) / CLOCKS_PER_SEC;
    cout << "partialCholesky " << nFrontal << ": ";
    auto ms = ((double)seconds * 1000000 / n);
    cout << ms << " ms, " << ms/nFrontal << " ms/dim" << endl;
  }

  return 0;
}
