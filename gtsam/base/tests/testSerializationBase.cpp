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

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;


/* ************************************************************************* */
TEST (Serialization, matrix_vector) {
  EXPECT(equality<Vector>(Vector_(4, 1.0, 2.0, 3.0, 4.0)));
  EXPECT(equality<Matrix>(Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0)));

  EXPECT(equalityXML<Vector>(Vector_(4, 1.0, 2.0, 3.0, 4.0)));
  EXPECT(equalityXML<Matrix>(Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0)));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
