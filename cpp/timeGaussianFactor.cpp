/**
 * @file    timeGaussianFactor.cpp
 * @brief   time GaussianFactor.eliminate
 * @author  Alireza Fathi
 */

#define GTSAM_MAGIC_KEY

#include <time.h>

/*STL/C++*/
#include <iostream>
using namespace std;

#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp> // for operator += in Ordering

#include "Matrix.h"
#include "GaussianFactor.h"
#include "GaussianConditional.h"
#include "Ordering.h"

using namespace gtsam;
using namespace boost::assign;

/*
 * Alex's Machine
 * Results for Eliminate:
 * Initial (1891): 17.91 sec, 55834.7 calls/sec
 *
 * Results for matrix_augmented:
 * Initial (1891)       :  0.85 sec, 1.17647e+06 calls/sec
 * int->size_t Version  :  8.45 sec (for n1 reps) with memcpy version of collect()
 * w/ original collect():  8.73 sec (for n1 reps)
 * b memcpy Version     :  8.64 sec (for n1 reps) with original version of collect()
 * w/ memcpy collect()  :  8.40 sec (for n1 reps)
 */

int main()
{
  // create a linear factor
  Matrix Ax2 = Matrix_(8,2,
		       // x2  
		       -5., 0.,
		       +0.,-5.,
		       10., 0.,
		       +0.,10.,
		       -5., 0.,
		       +0.,-5.,
		       10., 0.,
		       +0.,10.
		       );
                     
  Matrix Al1 = Matrix_(8,10,
		       // l1     
		       5., 0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0., 5.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0., 0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0., 0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       5., 0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0., 5.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0., 0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0., 0.,1.,2.,3.,4.,5.,6.,7.,8.
		       );
                     
  Matrix Ax1 = Matrix_(8,2,
		       // x1
		       0.00,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0.00,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       -10.,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0.00,-10.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0.00,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0.00,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       -10.,  0.,1.,2.,3.,4.,5.,6.,7.,8.,
		       0.00,-10.,1.,2.,3.,4.,5.,6.,7.,8.
		       );

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
  GaussianFactor combined("x2", Ax2,  "l1", Al1, "x1", Ax1, b2,1);
  long timeLog = clock();
  int n = 1000000;
  GaussianConditional::shared_ptr conditional;
  GaussianFactor::shared_ptr factor;

  for(int i = 0; i < n; i++)
    boost::tie(conditional,factor) = combined.eliminate("x2");

  long timeLog2 = clock();
  double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << "Single Eliminate Timing:" << endl;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  // time matrix_augmented
  Ordering ordering;
  ordering += "x2", "l1", "x1";
  size_t n1 = 10000000;
  timeLog = clock();

  for(int i = 0; i < n1; i++)
	  Matrix Ab = combined.matrix_augmented(ordering, true);

  timeLog2 = clock();
  seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << "matrix_augmented Timing:" << endl;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;

  return 0;
}
