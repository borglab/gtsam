/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSerializationInference.cpp
 * @brief
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#include <gtsam/symbolic/tests/symbolicExampleGraphs.h>

#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

/* ************************************************************************* */
TEST (Serialization, symbolic_graph) {
  EXPECT(equalsObj(asiaGraph));
  EXPECT(equalsXML(asiaGraph));
  EXPECT(equalsBinary(asiaGraph));
}

/* ************************************************************************* */
TEST (Serialization, symbolic_bn) {
  EXPECT(equalsObj(asiaBayesNet));
  EXPECT(equalsXML(asiaBayesNet));
  EXPECT(equalsBinary(asiaBayesNet));
}

/* ************************************************************************* */
TEST (Serialization, symbolic_bayes_tree ) {
  EXPECT(equalsObj(asiaBayesTree));
  EXPECT(equalsXML(asiaBayesTree));
  EXPECT(equalsBinary(asiaBayesTree));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
