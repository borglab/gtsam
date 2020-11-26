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

  BaseOptimizerParameters baseOptimizerParams;

  /// any other specific GNC parameters:
};

/* ************************************************************************* */
//template <class GncParameters>
//class GncOptimizer {
// public:
//  // types etc
//
// private:
//  FG INITIAL GncParameters params_;
//
// public:
//  GncOptimizer(FG, INITIAL, const GncParameters& params) : params(params) {
//    // Check that all noise models are Gaussian
//  }
//
//  Values optimize() const {
//    NonlinearFactorGraph currentGraph = graph_;
//    for (i : {1, 2, 3}) {
//      BaseOptimizer::Optimizer baseOptimizer(currentGraph, initial);
//      VALUES currentSolution = baseOptimizer.optimize();
//      if (converged) {
//        return currentSolution;
//      }
//      graph_i = this->makeGraph(currentSolution);
//    }
//  }
//
//  NonlinearFactorGraph makeGraph(const Values& currentSolution) const {
//    // calculate some weights
//    this->calculateWeights();
//    // copy the graph with new weights
//
//  }
//};

///* ************************************************************************* */
//TEST(GncOptimizer, calculateWeights) {
//}
//
///* ************************************************************************* */
//TEST(GncOptimizer, copyGraph) {
//}

/* ************************************************************************* */
TEST(GncOptimizer, makeGraph) {
  // has to have Gaussian noise models !
  auto fg = example::createReallyNonlinearFactorGraph(); // just a unary factor on a 2D point

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
//  auto gnc = GncOptimizer(fg, initial, gncParams);

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
