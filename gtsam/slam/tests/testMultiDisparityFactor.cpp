/*
 * testMultiDisparityFactor.cpp
 *
 *  Created on: Jan 31, 2014
 *      Author: nsrinivasan7
 *      @brief: Unittest for MultidisparityFactor
 */


#include <gtsam/geometry/Sphere2.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/MultiDisparityFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>

using namespace boost::assign;
using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(OrientedPlane3)
GTSAM_CONCEPT_MANIFOLD_INST(OrientedPlane3)

TEST(MutliDisparityFactor,error)
{

  Key key(1);
  Vector disparities = Vector_(2, 1.0, 1.0); // matlab generated values

  Eigen::Matrix<int,Eigen::Dynamic,2> uv;
  uv.resize(2,2);
  uv.block<2,2>(0,0) << 20, 30, 40, 60;
  SharedIsotropic model = gtsam::noiseModel::Isotropic::Sigma(disparities.rows(), 0.25, true);

  Pose3 cameraPose;

  MultiDisparityFactor factor(key, disparities, uv, cameraPose, model);
  factor.print("Multi Disparity factor");

}

/* ************************************************************************* */
int main() {
  srand(time(NULL));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
