/**
 * @file    simulated2D.cpp
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

#include "simulated2D.h"

namespace gtsam {


/** prior on a single pose */

/* ************************************************************************* */
Vector prior (const Vector& x) {return x;}

/* ************************************************************************* */
Matrix Dprior(const Vector& x) {
  Matrix H(2,2);
  H(0,0)=1;  H(0,1)=0;
  H(1,0)=0;  H(1,1)=1;
  return H;
}
/* ************************************************************************* */

/** odometry between two poses */                                

/* ************************************************************************* */
Vector odo(const Vector& x1, const Vector& x2) {return x2 - x1;}
Matrix Dodo1(const Vector& x1, const Vector& x2) {
  Matrix H(2,2);
  H(0,0)=-1;  H(0,1)= 0;
  H(1,0)= 0;  H(1,1)=-1;
  return H;
}

/* ************************************************************************* */
Matrix Dodo2(const Vector& x1, const Vector& x2) {
  Matrix H(2,2);
  H(0,0)= 1;  H(0,1)= 0;
  H(1,0)= 0;  H(1,1)= 1;
  return H;
}

/* ************************************************************************* */

/** measurement between landmark and pose */

/* ************************************************************************* */
Vector mea(const Vector& x,  const Vector& l)  {return l  - x;}
Matrix Dmea1(const Vector& x, const Vector& l) {
  Matrix H(2,2);
  H(0,0)=-1;  H(0,1)= 0;
  H(1,0)= 0;  H(1,1)=-1;
  return H;
}

/* ************************************************************************* */
Matrix Dmea2(const Vector& x, const Vector& l) {
  Matrix H(2,2);
  H(0,0)= 1;  H(0,1)= 0;
  H(1,0)= 0;  H(1,1)= 1;
  return H;
}

/* ************************************************************************* */
} // namespace gtsam
