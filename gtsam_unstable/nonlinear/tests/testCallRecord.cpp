/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testCallRecord.cpp
 * @date November 21, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @author Hannes Sommer
 * @brief unit tests for Callrecord class
 */

#include <gtsam_unstable/nonlinear/Callrecord.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
static const int Cols = 3;

struct Record: public internal::CallRecordImplementor<Record, Cols> {
  virtual ~Record() {
  }
  void print(const std::string& indent) const {
  }
  void startReverseAD(JacobianMap& jacobians) const {
  }
  template<typename SomeMatrix>
  void reverseAD(const SomeMatrix & dFdT, JacobianMap& jacobians) const {
  }
};

/* ************************************************************************* */
// Construct
TEST(CallRecord, constant) {
  Record record;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

