/*
 * testLieConfig.cpp
 *
 *  Created on: Jan 5, 2010
 *      Author: richard
 */

#include <CppUnitLite/TestHarness.h>
#include <stdexcept>
#include <Pose2.h>

#include "LieConfig-inl.h"
#include "Vector.h"

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST( LieConfig, equals1 )
{
  LieConfig<Vector> expected;
  Vector v = Vector_(3, 5.0, 6.0, 7.0);
  expected.insert("a",v);
  LieConfig<Vector> actual;
  actual.insert("a",v);
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( LieConfig, equals2 )
{
  LieConfig<Vector> cfg1, cfg2;
  Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
  Vector v2 = Vector_(3, 5.0, 6.0, 8.0);
  cfg1.insert("x", v1);
  cfg2.insert("x", v2);
  CHECK(!cfg1.equals(cfg2));
  CHECK(!cfg2.equals(cfg1));
}

/* ************************************************************************* */
TEST( LieConfig, equals_nan )
{
  LieConfig<Vector> cfg1, cfg2;
  Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
  Vector v2 = Vector_(3, 0.0/0.0, 0.0/0.0, 0.0/0.0);
  cfg1.insert("x", v1);
  cfg2.insert("x", v2);
  CHECK(!cfg1.equals(cfg2));
  CHECK(!cfg2.equals(cfg1));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_a)
{
  LieConfig<Vector> config0;
  config0.insert("v1", Vector_(3, 1.0, 2.0, 3.0));
  config0.insert("v2", Vector_(3, 5.0, 6.0, 7.0));

  VectorConfig increment;
  increment.insert("v1", Vector_(3, 1.0, 1.1, 1.2));
  increment.insert("v2", Vector_(3, 1.3, 1.4, 1.5));

  LieConfig<Vector> expected;
  expected.insert("v1", Vector_(3, 2.0, 3.1, 4.2));
  expected.insert("v2", Vector_(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, expmap(config0, increment)));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_b)
{
  LieConfig<Vector> config0;
  config0.insert("v1", Vector_(3, 1.0, 2.0, 3.0));
  config0.insert("v2", Vector_(3, 5.0, 6.0, 7.0));

  VectorConfig increment;
  increment.insert("v2", Vector_(3, 1.3, 1.4, 1.5));

  LieConfig<Vector> expected;
  expected.insert("v1", Vector_(3, 1.0, 2.0, 3.0));
  expected.insert("v2", Vector_(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, expmap(config0, increment)));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_c)
{
  LieConfig<Vector> config0;
  config0.insert("v1", Vector_(3, 1.0, 2.0, 3.0));
  config0.insert("v2", Vector_(3, 5.0, 6.0, 7.0));

  Vector increment = Vector_(6,
      1.0, 1.1, 1.2,
      1.3, 1.4, 1.5);

  LieConfig<Vector> expected;
  expected.insert("v1", Vector_(3, 2.0, 3.1, 4.2));
  expected.insert("v2", Vector_(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, expmap(config0, increment)));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_d)
{
  LieConfig<Vector> config0;
  config0.insert("v1", Vector_(3, 1.0, 2.0, 3.0));
  config0.insert("v2", Vector_(3, 5.0, 6.0, 7.0));
  //config0.print("config0");

  LieConfig<Pose2> poseconfig;
  poseconfig.insert("p1", Pose2(1,2,3));
  poseconfig.insert("p2", Pose2(0.3, 0.4, 0.5));
  //poseconfig.print("poseconfig");
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
