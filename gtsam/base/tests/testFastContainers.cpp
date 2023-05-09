/**
 * @file testFastContainers.cpp
 *
 * @brief Test for the Fast* containers that use boost pool allocators and interfaces
 *
 * @date Sep 24, 2012
 * @author Alex Cunningham
 */

#include <gtsam/inference/Key.h>
#include <gtsam/base/FastSet.h>
#include <gtsam/base/FastVector.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/std/set.hpp>

#include <CppUnitLite/TestHarness.h>

using namespace boost::assign;
using namespace gtsam;

/* ************************************************************************* */
TEST( testFastContainers, KeySet ) {

  KeyVector init_vector {2, 3, 4, 5};

  KeySet actSet(init_vector);
  KeySet expSet; expSet += 2, 3, 4, 5;
  EXPECT(actSet == expSet);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
