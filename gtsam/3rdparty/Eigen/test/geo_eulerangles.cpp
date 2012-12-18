// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "main.h"
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

template<typename Scalar> void eulerangles(void)
{
  typedef Matrix<Scalar,3,3> Matrix3;
  typedef Matrix<Scalar,3,1> Vector3;
  typedef Quaternion<Scalar> Quaternionx;
  typedef AngleAxis<Scalar> AngleAxisx;

  Scalar a = internal::random<Scalar>(-Scalar(M_PI), Scalar(M_PI));
  Quaternionx q1;
  q1 = AngleAxisx(a, Vector3::Random().normalized());
  Matrix3 m;
  m = q1;

  #define VERIFY_EULER(I,J,K, X,Y,Z) { \
    Vector3 ea = m.eulerAngles(I,J,K); \
    VERIFY_IS_APPROX(m,  Matrix3(AngleAxisx(ea[0], Vector3::Unit##X()) * AngleAxisx(ea[1], Vector3::Unit##Y()) * AngleAxisx(ea[2], Vector3::Unit##Z()))); \
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

void test_geo_eulerangles()
{
  for(int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1( eulerangles<float>() );
    CALL_SUBTEST_2( eulerangles<double>() );
  }
}
