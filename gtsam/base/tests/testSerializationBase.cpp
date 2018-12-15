/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSerializationBase.cpp
 * @brief
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#include <gtsam/inference/Key.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/FastList.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/FastSet.h>
#include <gtsam/base/FastVector.h>

#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

Vector v1 = Vector2(1.0, 2.0);
Vector v2 = Vector2(3.0, 4.0);
Vector v3 = Vector2(5.0, 6.0);

/* ************************************************************************* */
TEST (Serialization, FastList) {
  FastList<Vector> list;
  list.push_back(v1);
  list.push_back(v2);
  list.push_back(v3);

  EXPECT(equality(list));
  EXPECT(equalityXML(list));
  EXPECT(equalityBinary(list));
}

/* ************************************************************************* */
TEST (Serialization, FastMap) {
  FastMap<int, Vector> map;
  map.insert(make_pair(1, v1));
  map.insert(make_pair(2, v2));
  map.insert(make_pair(3, v3));

  EXPECT(equality(map));
  EXPECT(equalityXML(map));
  EXPECT(equalityBinary(map));
}

/* ************************************************************************* */
TEST (Serialization, FastSet) {
  KeySet set;
  set.insert(1);
  set.insert(2);
  set.insert(3);

  EXPECT(equality(set));
  EXPECT(equalityXML(set));
  EXPECT(equalityBinary(set));
}

/* ************************************************************************* */
TEST (Serialization, FastVector) {
  FastVector<Vector> vector;
  vector.push_back(v1);
  vector.push_back(v2);
  vector.push_back(v3);

  EXPECT(equality(vector));
  EXPECT(equalityXML(vector));
  EXPECT(equalityBinary(vector));
}

/* ************************************************************************* */
TEST (Serialization, matrix_vector) {
  EXPECT(equality<Vector>((Vector(4) << 1.0, 2.0, 3.0, 4.0).finished()));
  EXPECT(equality<Vector2>(Vector2(1.0, 2.0)));
  EXPECT(equality<Vector3>(Vector3(1.0, 2.0, 3.0)));
  EXPECT(equality<Vector6>((Vector6() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished()));
  EXPECT(equality<Matrix>((Matrix(2, 2) << 1.0, 2.0, 3.0, 4.0).finished()));

  EXPECT(equalityXML<Vector>((Vector(4) << 1.0, 2.0, 3.0, 4.0).finished()));
  EXPECT(equalityXML<Vector2>(Vector2(1.0, 2.0)));
  EXPECT(equalityXML<Vector3>(Vector3(1.0, 2.0, 3.0)));
  EXPECT(equalityXML<Vector6>((Vector6() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished()));
  EXPECT(equalityXML<Matrix>((Matrix(2, 2) << 1.0, 2.0, 3.0, 4.0).finished()));

  EXPECT(equalityBinary<Vector>((Vector(4) << 1.0, 2.0, 3.0, 4.0).finished()));
  EXPECT(equalityBinary<Vector2>(Vector2(1.0, 2.0)));
  EXPECT(equalityBinary<Vector3>(Vector3(1.0, 2.0, 3.0)));
  EXPECT(equalityBinary<Vector6>((Vector6() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished()));
  EXPECT(equalityBinary<Matrix>((Matrix(2, 2) << 1.0, 2.0, 3.0, 4.0).finished()));
}

/* ************************************************************************* */


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
