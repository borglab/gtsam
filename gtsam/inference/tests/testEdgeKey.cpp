/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testEdgeKey.cpp
 * @date Oct 24, 2024
 * @author: Frank Dellaert
 * @author: Akshay Krishnan
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/EdgeKey.h>

#include <sstream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(EdgeKey, Construction) {
  EdgeKey edge(1, 2);
  EXPECT(edge.i() == 1);
  EXPECT(edge.j() == 2);
}

/* ************************************************************************* */
TEST(EdgeKey, Equality) {
  EdgeKey edge1(1, 2);
  EdgeKey edge2(1, 2);
  EdgeKey edge3(2, 3);

  EXPECT(assert_equal(edge1, edge2));
  EXPECT(!edge1.equals(edge3));
}

/* ************************************************************************* */
TEST(EdgeKey, StreamOutput) {
  EdgeKey edge(1, 2);
  std::ostringstream oss;
  oss << edge;
  EXPECT("{1, 2}" == oss.str());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
