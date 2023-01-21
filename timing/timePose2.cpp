/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timePose2.cpp
 * @brief   Time Pose2 geometry
 * @author  Richard Roberts
 */

#include <iostream>

#include <gtsam/base/timing.h>
#include <gtsam/geometry/Pose2.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
#define TEST(TITLE,STATEMENT) \
  gttic_(TITLE); \
  for(int i = 0; i < n; i++) \
  STATEMENT; \
  gttoc_(TITLE);

/* ************************************************************************* */
Pose2 Pose2betweenDefault(const Pose2& r1, const Pose2& r2) {
  return r1.inverse() * r2;
}

/* ************************************************************************* */
Pose2 Pose2betweenOptimized(const Pose2& r1, const Pose2& r2,
  OptionalJacobian<3,3> H1 = {}, OptionalJacobian<3,3> H2 = {}) {
  // get cosines and sines from rotation matrices
  const Rot2& R1 = r1.r(), R2 = r2.r();
  double c1=R1.c(), s1=R1.s(), c2=R2.c(), s2=R2.s();

  // Assert that R1 and R2 are normalized
  assert(std::abs(c1*c1 + s1*s1 - 1.0) < 1e-5 && std::abs(c2*c2 + s2*s2 - 1.0) < 1e-5);

  // Calculate delta rotation = between(R1,R2)
  double c = c1 * c2 + s1 * s2, s = -s1 * c2 + c1 * s2;
  Rot2 R(Rot2::atan2(s,c)); // normalizes

  // Calculate delta translation = unrotate(R1, dt);
  Point2 dt = r2.t() - r1.t();
  double x = dt.x(), y = dt.y();
  Point2 t(c1 * x + s1 * y, -s1 * x + c1 * y);

  // FD: This is just -AdjointMap(between(p2,p1)) inlined and re-using above
  if (H1) {
    double dt1 = -s2 * x + c2 * y;
    double dt2 = -c2 * x - s2 * y;
    *H1 = (Matrix(3,3) <<
      -c,  -s,  dt1,
      s,  -c,  dt2,
      0.0, 0.0,-1.0).finished();
  }
  if (H2) *H2 = I_3x3;

  return Pose2(R,t);
}

/* ************************************************************************* */
Vector Pose2BetweenFactorEvaluateErrorDefault(const Pose2& measured, const Pose2& p1, const Pose2& p2,
  OptionalJacobian<3,3> H1, OptionalJacobian<3,3> H2)
{
  Pose2 hx = p1.between(p2, H1, H2); // h(x)
  // manifold equivalent of h(x)-z -> log(z,h(x))
  return measured.localCoordinates(hx);
}

/* ************************************************************************* */
Vector Pose2BetweenFactorEvaluateErrorOptimizedBetween(const Pose2& measured, const Pose2& p1, const Pose2& p2,
  OptionalJacobian<3,3> H1, OptionalJacobian<3,3> H2)
{
  Pose2 hx = Pose2betweenOptimized(p1, p2, H1, H2); // h(x)
  // manifold equivalent of h(x)-z -> log(z,h(x))
  return Pose2::Logmap(Pose2betweenOptimized(measured, hx));
}

/* ************************************************************************* */
Vector Pose2BetweenFactorEvaluateErrorOptimizedBetweenFixed(const Pose2& measured, const Pose2& p1, const Pose2& p2,
  OptionalJacobian<3,3> H1, OptionalJacobian<3,3> H2)
{
  // TODO: Implement
  Pose2 hx = Pose2betweenOptimized(p1, p2, H1, H2); // h(x)
  // manifold equivalent of h(x)-z -> log(z,h(x))
  return Pose2::Logmap(Pose2betweenOptimized(measured, hx));
}

/* ************************************************************************* */
int main()
{
  int n = 50000000;
  cout << "NOTE:  Times are reported for " << n << " calls" << endl;

  // create a random pose:
  double x = 4.0, y = 2.0, r = 0.3;
  Vector v = (Vector(3) << x, y, r).finished();
  Pose2 X = Pose2(3,2,0.4), X2 = X.retract(v), X3(5,6,0.3);

  TEST(Expmap, Pose2::Expmap(v));
  TEST(Retract, X.retract(v));
  TEST(Logmap, Pose2::Logmap(X2));
  TEST(localCoordinates, X.localCoordinates(X2));

  Matrix H1, H2;
  Matrix3 H1f, H2f;
  TEST(Pose2BetweenFactorEvaluateErrorDefault, Pose2BetweenFactorEvaluateErrorDefault(X3, X, X2, H1, H2));
  TEST(Pose2BetweenFactorEvaluateErrorOptimizedBetween, Pose2BetweenFactorEvaluateErrorOptimizedBetween(X3, X, X2, H1, H2));
  TEST(Pose2BetweenFactorEvaluateErrorOptimizedBetweenFixed, Pose2BetweenFactorEvaluateErrorOptimizedBetweenFixed(X3, X, X2, H1f, H2f));

  // Print timings
  tictoc_print_();

  return 0;
}
