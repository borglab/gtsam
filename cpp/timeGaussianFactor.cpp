/**
 * @file    timeGaussianFactor.cpp
 * @brief   time GaussianFactor.eliminate
 * @author  Alireza Fathi
 */

#include <time.h>

/*STL/C++*/
#include <iostream>
using namespace std;

#include <boost/tuple/tuple.hpp>
#include "Matrix.h"
#include "GaussianFactor.h"

using namespace gtsam;

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
  
  GaussianFactor combined("x2", Ax2,  "l1", Al1, "x1", Ax1, b2);
  long timeLog = clock();
  int n = 1000000;
  GaussianConditional::shared_ptr conditional;
  GaussianFactor::shared_ptr factor;

  for(int i = 0; i < n; i++)
    boost::tie(conditional,factor) = combined.eliminate("x2");

  long timeLog2 = clock();
  double seconds = (double)(timeLog2-timeLog)/CLOCKS_PER_SEC;
  cout << seconds << " seconds" << endl;
  cout << ((double)n/seconds) << " calls/second" << endl;
  return 0;
}
