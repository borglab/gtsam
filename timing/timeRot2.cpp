/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeRot2.cpp
 * @brief   Time Rot2 geometry
 * @author  Richard Roberts
 */


#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/timing.h>
#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
#define TEST(TITLE,STATEMENT) \
  gttic_(TITLE); \
  for(int i = 0; i < n; i++) \
  STATEMENT; \
  gttoc_(TITLE);

/* ************************************************************************* */
Rot2 Rot2betweenDefault(const Rot2& r1, const Rot2& r2) {
  return r1.inverse() * r2;
}

/* ************************************************************************* */
Rot2 Rot2betweenOptimized(const Rot2& r1, const Rot2& r2) {
  // Same as compose but sign of sin for r1 is reversed
  return Rot2::fromCosSin(r1.c() * r2.c() + r1.s() * r2.s(), -r1.s() * r2.c() + r1.c() * r2.s());
}

/* ************************************************************************* */
Vector Rot2BetweenFactorEvaluateErrorDefault(const Rot2& measured, const Rot2& p1, const Rot2& p2,
  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2)
{
  Rot2 hx = p1.between(p2, H1, H2); // h(x)
  // manifold equivalent of h(x)-z -> log(z,h(x))
  return measured.localCoordinates(hx);
}

/* ************************************************************************* */
Vector Rot2BetweenFactorEvaluateErrorOptimizedBetween(const Rot2& measured, const Rot2& p1, const Rot2& p2,
  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2)
{
  Rot2 hx = Rot2betweenOptimized(p1, p2); // h(x)
  if (H1) *H1 = -I_1x1;
  if (H2) *H2 = I_1x1;
  // manifold equivalent of h(x)-z -> log(z,h(x))
  return Rot2::Logmap(Rot2betweenOptimized(measured, hx));
}

/* ************************************************************************* */
Vector Rot2BetweenFactorEvaluateErrorOptimizedBetweenNoeye(const Rot2& measured, const Rot2& p1, const Rot2& p2,
  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2)
{
  // TODO: Implement
  Rot2 hx = Rot2betweenOptimized(p1, p2); // h(x)
  if (H1) *H1 = -I_1x1;
  if (H2) *H2 = I_1x1;
  // manifold equivalent of h(x)-z -> log(z,h(x))
  return Rot2::Logmap(Rot2betweenOptimized(measured, hx));
}

/* ************************************************************************* */
typedef Eigen::Matrix<double,1,1> Matrix1;
Vector Rot2BetweenFactorEvaluateErrorOptimizedBetweenFixed(const Rot2& measured, const Rot2& p1, const Rot2& p2,
  boost::optional<Matrix1&> H1, boost::optional<Matrix1&> H2)
{
  // TODO: Implement
  Rot2 hx = Rot2betweenOptimized(p1, p2); // h(x)
  if (H1) *H1 = -Matrix1::Identity();
  if (H2) *H2 = Matrix1::Identity();
  // manifold equivalent of h(x)-z -> log(z,h(x))
  return Rot2::Logmap(Rot2betweenOptimized(measured, hx));
}

/* ************************************************************************* */
int main()
{
  int n = 50000000;
  cout << "NOTE:  Times are reported for " << n << " calls" << endl;

  Vector1 v; v << 0.1;
  Rot2 R = Rot2(0.4), R2(0.5), R3(0.6);

  TEST(Rot2_Expmap, Rot2::Expmap(v));
  TEST(Rot2_Retract, R.retract(v));
  TEST(Rot2_Logmap, Rot2::Logmap(R2));
  TEST(Rot2_between, R.between(R2));
  TEST(betweenDefault, Rot2betweenDefault(R, R2));
  TEST(betweenOptimized, Rot2betweenOptimized(R, R2));
  TEST(Rot2_localCoordinates, R.localCoordinates(R2));

  Matrix H1, H2;
  Matrix1 H1f, H2f;
  TEST(Rot2BetweenFactorEvaluateErrorDefault, Rot2BetweenFactorEvaluateErrorDefault(R3, R, R2, H1, H2));
  TEST(Rot2BetweenFactorEvaluateErrorOptimizedBetween, Rot2BetweenFactorEvaluateErrorOptimizedBetween(R3, R, R2, H1, H2));
  TEST(Rot2BetweenFactorEvaluateErrorOptimizedBetweenNoeye, Rot2BetweenFactorEvaluateErrorOptimizedBetweenNoeye(R3, R, R2, H1, H2));
  TEST(Rot2BetweenFactorEvaluateErrorOptimizedBetweenFixed, Rot2BetweenFactorEvaluateErrorOptimizedBetweenFixed(R3, R, R2, H1f, H2f));

  // Print timings
  tictoc_print_();

  return 0;
}
