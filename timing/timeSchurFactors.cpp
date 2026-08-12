/**
 * @file    timeSchurFactors.cpp
 * @brief   Time various Schur-complement Jacobian factors
 * @author  Frank Dellaert
 * @date    Oct 27, 2013
 */

#include "DummyFactor.h"
#include <gtsam/base/timing.h>

#include <gtsam/slam/JacobianFactorQ.h>
#include "gtsam/slam/JacobianFactorQR.h"
#include <gtsam/slam/RegularImplicitSchurFactor.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholePose.h>

#include <fstream>

using namespace std;
using namespace gtsam;

#define SLOW
#define RAW
#define HESSIAN
#define NUM_ITERATIONS 1000

// Create CSV file for results
ofstream os("timeSchurFactors.csv");

/*************************************************************************************/
template<typename CAMERA>
void timeAll(size_t m, size_t N) {

  cout << m << endl;

  // create F
  static const int D = CAMERA::dimension;
  typedef Eigen::Matrix<double, 2, D> Matrix2D;
  KeyVector keys;
  vector <Matrix2D, Eigen::aligned_allocator<Matrix2D>> Fblocks;
  for (size_t i = 0; i < m; i++) {
    keys.push_back(i);
    Fblocks.push_back((i + 1) * Matrix::Ones(2, D));
  }

  // create E
  Matrix E(2 * m, 3);
  for (size_t i = 0; i < m; i++)
    E.block < 2, 3 > (2 * i, 0) = Matrix::Ones(2, 3);

  // Calculate point covariance
  Matrix P = (E.transpose() * E).inverse();

  // RHS and sigmas
  const Vector b = Vector::Constant(2*m,1);
  const SharedDiagonal model;

  // parameters for multiplyHessianAdd
  double alpha = 0.5;
  VectorValues xvalues, yvalues;
  for (size_t i = 0; i < m; i++)
    xvalues.insert(i, Vector::Constant(D,2));

  // Implicit
  RegularImplicitSchurFactor<CAMERA> implicitFactor(keys, Fblocks, E, P, b);
  // JacobianFactor with same error
  JacobianFactorQ<D, 2> jf(keys, Fblocks, E, P, b, model);
  // JacobianFactorQR with same error
  JacobianFactorQR<D, 2> jqr(keys, Fblocks, E, P, b, model);
  // Hessian
  HessianFactor hessianFactor(jqr);

#define TIME(label,factor,xx,yy) {\
     for (size_t t = 0; t < N; t++) \
     factor.multiplyHessianAdd(alpha, xx, yy);\
     gttic_(label);\
     for (size_t t = 0; t < N; t++) {\
       factor.multiplyHessianAdd(alpha, xx, yy);\
     }\
     gttoc_(label);\
     tictoc_getNode(timer, label)\
     os << timer->secs()/NUM_ITERATIONS << ", ";\
   }

#ifdef SLOW
  TIME(Implicit, implicitFactor, xvalues, yvalues)
  TIME(Jacobian, jf, xvalues, yvalues)
  TIME(JacobianQR, jqr, xvalues, yvalues)
#endif

#ifdef HESSIAN
  TIME(Hessian, hessianFactor, xvalues, yvalues)
#endif

#ifdef OVERHEAD
  DummyFactor<D> dummy(Fblocks, E, P, b);
  TIME(Overhead,dummy,xvalues,yvalues)
#endif

#ifdef RAW
  { // Raw memory Version
    FastVector < Key > keys;
    for (size_t i = 0; i < m; i++)
      keys.push_back(i);
    Vector x = xvalues.vector(keys);
    double* xdata = x.data();

    // create a y
    Vector y = Vector::Zero(m * D);
    TIME(RawImplicit, implicitFactor, xdata, y.data())
    TIME(RawJacobianQ, jf, xdata, y.data())
    TIME(RawJacobianQR, jqr, xdata, y.data())
  }
#endif

  os << m << endl;

} // timeAll

/*************************************************************************************/
int main(void) {
#ifdef SLOW
  os << "Implicit,";
  os << "JacobianQ,";
  os << "JacobianQR,";
#endif
#ifdef HESSIAN
  os << "Hessian,";
#endif
#ifdef OVERHEAD
  os << "Overhead,";
#endif
#ifdef RAW
  os << "RawImplicit,";
  os << "RawJacobianQ,";
  os << "RawJacobianQR,";
#endif
  os << "m" << endl;
  // define images
  vector < size_t > ms;
  //  ms += 2;
  //  ms += 3, 4, 5, 6, 7, 8, 9, 10;
  // ms += 11,12,13,14,15,16,17,18,19;
  //  ms += 20, 30, 40, 50;
  // ms += 20,30,40,50,60,70,80,90,100;
  for (size_t m = 2; m <= 50; m += 2)
    ms.push_back(m);
  //for (size_t m=10;m<=100;m+=10) ms += m;
  // loop over number of images
  for(size_t m: ms)
    timeAll<PinholePose<Cal3Bundler> >(m, NUM_ITERATIONS);
}

//*************************************************************************************
