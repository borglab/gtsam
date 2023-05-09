/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testWhiteNoiseFactor.cpp
 * @author Frank Dellaert
 */

#include <gtsam/nonlinear/WhiteNoiseFactor.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST( WhiteNoiseFactor, constructor )
{
  double z = 0.1;
  Key meanKey=1, precisionKey=2;
  WhiteNoiseFactor factor(z,meanKey, precisionKey);
  LONGS_EQUAL(2, (long)factor.dim());
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
