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
 * @brief unit tests for CallRecord class
 */

#include <gtsam/nonlinear/internal/CallRecord.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
static const int Cols = 3;


int dynamicIfAboveMax(int i){
  if(i > internal::CallRecordMaxVirtualStaticRows){
    return Eigen::Dynamic;
  }
  else return i;
}
struct CallConfig {
  int compTimeRows;
  int compTimeCols;
  int runTimeRows;
  int runTimeCols;
  CallConfig(int rows, int cols):
    compTimeRows(dynamicIfAboveMax(rows)),
    compTimeCols(cols),
    runTimeRows(rows),
    runTimeCols(cols)
  {
  }
  CallConfig(int compTimeRows, int compTimeCols, int runTimeRows, int runTimeCols):
    compTimeRows(compTimeRows),
    compTimeCols(compTimeCols),
    runTimeRows(runTimeRows),
    runTimeCols(runTimeCols)
  {
  }

  bool equals(const CallConfig & c, double /*tol*/) const {
    return
        this->compTimeRows == c.compTimeRows &&
        this->compTimeCols == c.compTimeCols &&
        this->runTimeRows == c.runTimeRows &&
        this->runTimeCols == c.runTimeCols;
  }
  void print(const std::string & prefix) const {
    std::cout << prefix << "{" << compTimeRows << ", " << compTimeCols << ", " << runTimeRows << ", " << runTimeCols << "}\n" ;
  }
};

/// traits
namespace gtsam {
template<> struct traits<CallConfig> : public Testable<CallConfig> {};
}

struct Record: public internal::CallRecordImplementor<Record, Cols> {
  Record() : cc(0, 0) {}
  virtual ~Record() {
  }
  void print(const std::string& indent) const {
  }
  void startReverseAD4(internal::JacobianMap& jacobians) const {
  }

  mutable CallConfig cc;
 private:
  template<typename SomeMatrix>
  void reverseAD4(const SomeMatrix & dFdT, internal::JacobianMap& jacobians) const {
    cc.compTimeRows = SomeMatrix::RowsAtCompileTime;
    cc.compTimeCols = SomeMatrix::ColsAtCompileTime;
    cc.runTimeRows = dFdT.rows();
    cc.runTimeCols = dFdT.cols();
  }

  template<typename Derived, int Rows>
  friend struct internal::CallRecordImplementor;
};

internal::JacobianMap & NJM= *static_cast<internal::JacobianMap *>(nullptr);

/* ************************************************************************* */
typedef Eigen::Matrix<double, Eigen::Dynamic, Cols> DynRowMat;

TEST(CallRecord, virtualReverseAdDispatching) {
  Record record;
  {
    const int Rows = 1;
    record.CallRecord::reverseAD2(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD2(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD2(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
  {
    const int Rows = 2;
    record.CallRecord::reverseAD2(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD2(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD2(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
  {
    const int Rows = 3;
    record.CallRecord::reverseAD2(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD2(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD2(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
  {
    const int Rows = 4;
    record.CallRecord::reverseAD2(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD2(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD2(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
  {
    const int Rows = 5;
    record.CallRecord::reverseAD2(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD2(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD2(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
  {
    const int Rows = 6;
    record.CallRecord::reverseAD2(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD2(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD2(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

