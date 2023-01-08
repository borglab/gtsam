/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeGaussianFactor.cpp
 * @brief   time JacobianFactor.eliminate
 * @author  Alireza Fathi
 */

#include <time.h>

/*STL/C++*/
#include <iostream>
using namespace std;

#include <boost/tuple/tuple.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/NoiseModel.h>

using namespace gtsam;

static const Key _x1_=1, _x2_=2, _l1_=3;

/*
 * Alex's Machine
 * Results for Eliminate:
 * Initial (1891): 17.91 sec, 55834.7 calls/sec
 * NoiseQR       : 11.69 sec
 *
 * Results for matrix_augmented:
 * Initial (1891)       :  0.85 sec, 1.17647e+06 calls/sec
 * int->size_t Version  :  8.45 sec (for n1 reps) with memcpy version of collect()
 * w/ original collect():  8.73 sec (for n1 reps)
 * b memcpy Version     :  8.64 sec (for n1 reps) with original version of collect()
 * w/ memcpy collect()  :  8.40 sec (for n1 reps)
 * Rev 2100             :  8.15 sec
 */

int main()
{
  // create a linear factor
  Matrix Ax2 = (Matrix(8, 2) <<
           // x2
           -5., 0.,
           +0.,-5.,
           10., 0.,
           +0.,10.,
           -5., 0.,
           +0.,-5.,
           10., 0.,
           +0.,10.
           ).finished();

  Matrix Al1 = (Matrix(8, 10) <<
           // l1
           5., 0.,1.,2.,3.,4.,5.,6.,7.,8.,
           0., 5.,1.,2.,3.,4.,5.,6.,7.,8.,
           0., 0.,1.,2.,3.,4.,5.,6.,7.,8.,
           0., 0.,1.,2.,3.,4.,5.,6.,7.,8.,
           5., 0.,1.,2.,3.,4.,5.,6.,7.,8.,
           0., 5.,1.,2.,3.,4.,5.,6.,7.,8.,
           0., 0.,1.,2.,3.,4.,5.,6.,7.,8.,
           0., 0.,1.,2.,3.,4.,5.,6.,7.,8.
           ).finished();

  Matrix Ax1 = (Matrix(8, 2) <<
           // x1
           0.00,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
           0.00,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
           -10.,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
           0.00,-10.,1.,2.,3.,4.,5.,6.,7.,8.,
           0.00,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
           0.00,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
           -10.,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
           0.00,-10.,1.,2.,3.,4.,5.,6.,7.,8.
           ).finished();

  // and a RHS
  Vector b2(8);
  b2(0) = -1;
  b2(1) = 1.5;
  b2(2) = 2;
  b2(3) = -1;
  b2(4) = -1;
  b2(5) = 1.5;
  b2(6) = 2;
  b2(7) = -1;

  // time eliminate
  JacobianFactor combined(_x2_, Ax2,  _l1_, Al1, _x1_, Ax1, b2, noiseModel::Isotropic::Sigma(8,1));
  long timeLog = clock();
  int n = 1000000;
  GaussianConditional::shared_ptr conditional;
  JacobianFactor::shared_ptr factor;

  for(int i = 0; i < n; i++)
    boost::tie(conditional, factor) =
        JacobianFactor(combined).eliminate(Ordering{_x2_});

  long timeLog2 = clock();
  double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << "Single Eliminate Timing:" << endl;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  // time matrix_augmented
//  Ordering ordering;
//  ordering += _x2_, _l1_, _x1_;
//  size_t n1 = 10000000;
//  timeLog = clock();
//
//  for(size_t i = 0; i < n1; i++)
//    Matrix Ab = combined.matrix_augmented(ordering, true);
//
//  timeLog2 = clock();
//  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
//  cout << "matrix_augmented Timing:" << endl;
//  cout << seconds << " seconds" << endl;
//  cout << ((double)n/seconds) << " calls/second" << endl;

  return 0;
}
