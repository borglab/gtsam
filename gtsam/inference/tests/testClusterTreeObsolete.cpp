/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testClusterTree.cpp
 * @brief   Unit tests for Bayes Tree
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/SymbolicFactorGraphOrdered.h>
#include <gtsam/inference/ClusterTreeOrdered.h>

using namespace gtsam;

// explicit instantiation and typedef
namespace gtsam { template class ClusterTreeOrdered<SymbolicFactorGraphOrdered>; }
typedef ClusterTreeOrdered<SymbolicFactorGraphOrdered> SymbolicClusterTree;

/* ************************************************************************* */
TEST(ClusterTreeOrdered, constructor) {
  SymbolicClusterTree tree;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
