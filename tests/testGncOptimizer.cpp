/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGncOptimizer.cpp
 * @brief   Unit tests for GncOptimizer class
 * @author  Jingnan Shi
 * @author  Luca Carlone
 * @author  Frank Dellaert
 */

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <tests/smallExample.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
template <class BaseOptimizerParameters>
class GncParams {
public:

  // using BaseOptimizer = BaseOptimizerParameters::OptimizerType;
  GncParams(const BaseOptimizerParameters& baseOptimizerParams): baseOptimizerParams(baseOptimizerParams) {}

  // default constructor
  GncParams(): baseOptimizerParams() {}

  BaseOptimizerParameters baseOptimizerParams;

  /// any other specific GNC parameters:
};

/* ************************************************************************* */
template<class GncParameters>
class GncOptimizer {
public:
  // types etc

private:
  NonlinearFactorGraph nfg_;
  Values state_;
  GncParameters params_;

public:
  GncOptimizer(const NonlinearFactorGraph& graph,
      const Values& initialValues, const GncParameters& params = GncParameters()) :
        nfg_(graph), state_(initialValues), params_(params) {
    // TODO: Check that all noise models are Gaussian
  }

//  Values optimize() const {
//    NonlinearFactorGraph currentGraph = graph_;
//  for (i : {1, 2, 3}) {
//    BaseOptimizer::Optimizer baseOptimizer(currentGraph, initial);
//    VALUES currentSolution = baseOptimizer.optimize();
//    if (converged) {
//      return currentSolution;
//    }
//    graph_i = this->makeGraph(currentSolution);
//  }
//}

//NonlinearFactorGraph makeGraph(const Values& currentSolution) const {
//  // calculate some weights
//  this->calculateWeights();
//  // copy the graph with new weights
//
//}
};

///* ************************************************************************* */
//TEST(GncOptimizer, calculateWeights) {
//}
//
///* ************************************************************************* */
//TEST(GncOptimizer, copyGraph) {
//}

/* ************************************************************************* */
TEST(GncOptimizer, gncParamsConstructor) {

  //check params are correctly parsed
  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams1(lmParams);
  CHECK(lmParams.equals(gncParams1.baseOptimizerParams));

  // check also default constructor
  GncParams<LevenbergMarquardtParams> gncParams1b;
  CHECK(lmParams.equals(gncParams1b.baseOptimizerParams));

  // and check params become different if we change lmParams
  lmParams.setVerbosity("DELTA");
  CHECK(!lmParams.equals(gncParams1.baseOptimizerParams));

  // and same for GN
  GaussNewtonParams gnParams;
  GncParams<GaussNewtonParams> gncParams2(gnParams);
  CHECK(gnParams.equals(gncParams2.baseOptimizerParams));

  // check default constructor
  GncParams<GaussNewtonParams> gncParams2b;
  CHECK(gnParams.equals(gncParams2b.baseOptimizerParams));
}

/* ************************************************************************* */
TEST(GncOptimizer, makeGraph) {
  // has to have Gaussian noise models !
  auto fg = example::createReallyNonlinearFactorGraph(); // just a unary factor on a 2D point

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(fg, initial, gncParams);

//  NonlinearFactorGraph actual = gnc.makeGraph(initial);
}

/* ************************************************************************* *
TEST(GncOptimizer, optimize) {
  // has to have Gaussian noise models !
  auto fg = example::createReallyNonlinearFactorGraph();

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams gncParams(lmParams);
  auto gnc = GncOptimizer(fg, initial, gncParams);
  Values actual = gnc.optimize();
  DOUBLES_EQUAL(0, fg.error(actual2), tol);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
