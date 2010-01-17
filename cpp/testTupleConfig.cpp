/*
 * testTupleConfig.cpp
 *
 *  Created on: Jan 13, 2010
 *      Author: richard
 */

#include <CppUnitLite/TestHarness.h>
#include <stdexcept>

#define GTSAM_MAGIC_KEY

#include <Pose2.h>
#include <Point2.h>

#include "Vector.h"
#include "Key.h"
#include "VectorConfig.h"
#include "TupleConfig-inl.h"

using namespace gtsam;
using namespace std;

typedef TypedSymbol<Pose2, 'x'> PoseKey;
typedef TypedSymbol<Point2, 'l'> PointKey;
typedef PairConfig<PoseKey, Pose2, PointKey, Point2> Config;

/* ************************************************************************* */
TEST( PairConfig, insert_equals1 )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config expected;
  expected.insert(1, x1);
  expected.insert(2, x2);
  expected.insert(1, l1);
  expected.insert(2, l2);

  Config actual;
  actual.insert(1, x1);
  actual.insert(2, x2);
  actual.insert(1, l1);
  actual.insert(2, l2);

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( PairConfig, insert_equals2 )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config cfg1;
  cfg1.insert(1, x1);
  cfg1.insert(2, x2);
  cfg1.insert(1, l1);
  cfg1.insert(2, l2);

  Config cfg2;
  cfg2.insert(1, x1);
  cfg2.insert(2, x2);
  cfg2.insert(1, l1);

  CHECK(!cfg1.equals(cfg2));

  cfg2.insert(2, Point2(9,11));

  CHECK(!cfg1.equals(cfg2));
}

///* ************************************************************************* */
//TEST( PairConfig, insert_duplicate )
//{
//  Pose2 x1(1,2,3), x2(6,7,8);
//  Point2 l1(4,5), l2(9,10);
//
//  Config cfg1;
//  cfg1.insert(1, x1);
//  cfg1.insert(2, x2);
//  cfg1.insert(1, l1);
//  cfg1.insert(2, l2);
//  cfg1.insert(2, l1);
//
//  CHECK(assert_equal(l2, cfg1[PointKey(2)]));
//  CHECK(cfg1.size() == 4);
//  CHECK(cfg1.dim() == 10);
//}


/* ************************************************************************* */
TEST( PairConfig, size_dim )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config cfg1;
  cfg1.insert(1, x1);
  cfg1.insert(2, x2);
  cfg1.insert(1, l1);
  cfg1.insert(2, l2);

  CHECK(cfg1.size() == 4);
  CHECK(cfg1.dim() == 10);
}

/* ************************************************************************* */
TEST(PairConfig, at)
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config cfg1;
  cfg1.insert(1, x1);
  cfg1.insert(2, x2);
  cfg1.insert(1, l1);
  cfg1.insert(2, l2);

  CHECK(assert_equal(x1, cfg1[PoseKey(1)]));
  CHECK(assert_equal(x2, cfg1[PoseKey(2)]));
  CHECK(assert_equal(l1, cfg1[PointKey(1)]));
  CHECK(assert_equal(l2, cfg1[PointKey(2)]));

  bool caught = false;
  try {
    cfg1[PoseKey(3)];
  } catch(invalid_argument e) {
    caught = true;
  }
  CHECK(caught);

  caught = false;
  try {
    cfg1[PointKey(3)];
  } catch(invalid_argument e) {
    caught = true;
  }
  CHECK(caught);
}

/* ************************************************************************* */
TEST(PairConfig, expmap)
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config cfg1;
  cfg1.insert(1, x1);
  cfg1.insert(2, x2);
  cfg1.insert(1, l1);
  cfg1.insert(2, l2);

  VectorConfig increment;
  increment.insert("x1", Vector_(3, 1.0, 1.1, 1.2));
  increment.insert("x2", Vector_(3, 1.3, 1.4, 1.5));
  increment.insert("l1", Vector_(2, 1.0, 1.1));
  increment.insert("l2", Vector_(2, 1.3, 1.4));

  Config expected;
  expected.insert(1, expmap(x1, Vector_(3, 1.0, 1.1, 1.2)));
  expected.insert(2, expmap(x2, Vector_(3, 1.3, 1.4, 1.5)));
  expected.insert(1, Point2(5.0, 6.1));
  expected.insert(2, Point2(10.3, 11.4));

  CHECK(assert_equal(expected, expmap(cfg1, increment)));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
