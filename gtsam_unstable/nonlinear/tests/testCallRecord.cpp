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

#include <gtsam_unstable/nonlinear/CallRecord.h>

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
static const int Cols = 3;


int dynamicIfAboveMax(int i){
  if(i > MaxVirtualStaticRows){
    return Eigen::Dynamic;
  }
  else return i;
}
struct CallConfig {
  int compTimeRows;
  int compTimeCols;
  int runTimeRows;
  int runTimeCols;
  CallConfig() {}
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

struct Record: public internal::CallRecordImplementor<Record, Cols> {
  virtual ~Record() {
  }
  void print(const std::string& indent) const {
  }
  void startReverseAD(JacobianMap& jacobians) const {
  }

  mutable CallConfig cc;
 private:
  template<typename SomeMatrix>
  void reverseAD(const SomeMatrix & dFdT, JacobianMap& jacobians) const {
    cc.compTimeRows = SomeMatrix::RowsAtCompileTime;
    cc.compTimeCols = SomeMatrix::ColsAtCompileTime;
    cc.runTimeRows = dFdT.rows();
    cc.runTimeCols = dFdT.cols();
  }

  template<typename Derived, int Rows, int OtherCols>
  friend struct internal::ReverseADImplementor;
};

JacobianMap & NJM= *static_cast<JacobianMap *>(NULL);

/* ************************************************************************* */
typedef Eigen::Matrix<double, Eigen::Dynamic, Cols> DynRowMat;

TEST(CallRecord, virtualReverseAdDispatching) {
  Record record;
  {
    const int Rows = 1;
    record.CallRecord::reverseAD(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
  {
    const int Rows = 2;
    record.CallRecord::reverseAD(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
  {
    const int Rows = 3;
    record.CallRecord::reverseAD(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
  {
    const int Rows = MaxVirtualStaticRows;
    record.CallRecord::reverseAD(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
  {
    const int Rows = MaxVirtualStaticRows + 1;
    record.CallRecord::reverseAD(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
  {
    const int Rows = MaxVirtualStaticRows + 2;
    record.CallRecord::reverseAD(Eigen::Matrix<double, Rows, Cols>(), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Rows, Cols))));
    record.CallRecord::reverseAD(DynRowMat(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Cols, Rows, Cols))));
    record.CallRecord::reverseAD(Eigen::MatrixXd(Rows, Cols), NJM);
    EXPECT((assert_equal(record.cc, CallConfig(Eigen::Dynamic, Eigen::Dynamic, Rows, Cols))));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

