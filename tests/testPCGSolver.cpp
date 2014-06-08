/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testPCGSolver.cpp
 * @brief   Unit tests for PCGSolver class
 * @author  Yong-Dian Jian
 */

#include <tests/smallExample.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Matrix.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <iostream>
#include <fstream>

using namespace std;
using namespace gtsam;

const double tol = 1e-3;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( NonlinearOptimizer, optimization_method )
{
  LevenbergMarquardtParams paramsPCG;
  paramsPCG.linearSolverType = LevenbergMarquardtParams::Iterative;
  PCGSolverParameters::shared_ptr pcg = boost::make_shared<PCGSolverParameters>();
  pcg->preconditioner_ = boost::make_shared<DummyPreconditionerParameters>();
  paramsPCG.iterativeParams = pcg;

  NonlinearFactorGraph fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(10,10);
  Values c0;
  c0.insert(X(1), x0);

  Values actualPCG = LevenbergMarquardtOptimizer(fg, c0, paramsPCG).optimize();

  DOUBLES_EQUAL(0,fg.error(actualPCG),tol);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

