// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2012 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "main.h"
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

template<typename Scalar> void check_all_var(const Matrix<Scalar,3,1>& ea)
{
  typedef Matrix<Scalar,3,3> Matrix3;
  typedef Matrix<Scalar,3,1> Vector3;
  typedef AngleAxis<Scalar> AngleAxisx;
  using std::abs;
  
  #define VERIFY_EULER(I,J,K, X,Y,Z) { \
    Matrix3 m(AngleAxisx(ea[0], Vector3::Unit##X()) * AngleAxisx(ea[1], Vector3::Unit##Y()) * AngleAxisx(ea[2], Vector3::Unit##Z())); \
    Vector3 eabis = m.eulerAngles(I,J,K); \
    Matrix3 mbis(AngleAxisx(eabis[0], Vector3::Unit##X()) * AngleAxisx(eabis[1], Vector3::Unit##Y()) * AngleAxisx(eabis[2], Vector3::Unit##Z())); \
    VERIFY_IS_APPROX(m,  mbis); \
    /* If I==K, and ea[1]==0, then there no unique solution. */ \
    /* The remark apply in the case where I!=K, and |ea[1]| is close to pi/2. */ \
    if( (I!=K || ea[1]!=0) && (I==K || !internal::isApprox(abs(ea[1]),Scalar(M_PI/2),test_precision<Scalar>())) ) VERIFY((ea-eabis).norm() <= test_precision<Scalar>()); \
  }
  VERIFY_EULER(0,1,2, X,Y,Z);
  VERIFY_EULER(0,1,0, X,Y,X);
  VERIFY_EULER(0,2,1, X,Z,Y);
  VERIFY_EULER(0,2,0, X,Z,X);

  VERIFY_EULER(1,2,0, Y,Z,X);
  VERIFY_EULER(1,2,1, Y,Z,Y);
  VERIFY_EULER(1,0,2, Y,X,Z);
  VERIFY_EULER(1,0,1, Y,X,Y);

  VERIFY_EULER(2,0,1, Z,X,Y);
  VERIFY_EULER(2,0,2, Z,X,Z);
  VERIFY_EULER(2,1,0, Z,Y,X);
  VERIFY_EULER(2,1,2, Z,Y,Z);
}

template<typename Scalar> void eulerangles()
{
  typedef Matrix<Scalar,3,3> Matrix3;
  typedef Matrix<Scalar,3,1> Vector3;
  typedef Array<Scalar,3,1> Array3;
  typedef Quaternion<Scalar> Quaternionx;
  typedef AngleAxis<Scalar> AngleAxisx;

  Scalar a = internal::random<Scalar>(-Scalar(M_PI), Scalar(M_PI));
  Quaternionx q1;
  q1 = AngleAxisx(a, Vector3::Random().normalized());
  Matrix3 m;
  m = q1;
  
  Vector3 ea = m.eulerAngles(0,1,2);
  check_all_var(ea);
  ea = m.eulerAngles(0,1,0);
  check_all_var(ea);
  
  ea = (Array3::Random() + Array3(1,1,0))*Scalar(M_PI)*Array3(0.5,0.5,1);
  check_all_var(ea);
  
  ea[2] = ea[0] = internal::random<Scalar>(0,Scalar(M_PI));
  check_all_var(ea);
  
  ea[0] = ea[1] = internal::random<Scalar>(0,Scalar(M_PI));
  check_all_var(ea);
  
  ea[1] = 0;
  check_all_var(ea);
  
  ea.head(2).setZero();
  check_all_var(ea);
  
  ea.setZero();
  check_all_var(ea);
}

void test_geo_eulerangles()
{
  for(int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1( eulerangles<float>() );
    CALL_SUBTEST_2( eulerangles<double>() );
  }
}
