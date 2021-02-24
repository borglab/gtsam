/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testConstantVelocityFactor.cpp
 * @brief   Unit test for ConstantVelocityFactor
 * @author  Alex Cunningham
 * @author  Asa Hammond
 */

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ConstantVelocityFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>
#include <list>

/* ************************************************************************* */
TEST(ConstantVelocityFactor, VelocityFactor) {
    using namespace gtsam;

    const auto tol = double{1e-5};

    const auto x1 = Key{1};
    const auto x2 = Key{2};

    const auto dt = double{1.0};
    const auto mu = double{1000};

    // moving upward with groundtruth velocity"
    const auto origin = NavState{Pose3{Rot3::Yaw(0), Point3{0.0, 0.0, 0.0}}, Velocity3{0.0, 0.0, 0.0}};

    const auto state0 = NavState{Pose3{Rot3::Yaw(0), Point3{0.0, 0.0, 0.0}}, Velocity3{0.0, 0.0, 1.0}};

    const auto state1 = NavState{Pose3{Rot3::Yaw(0), Point3{0.0, 0.0, 1.0}}, Velocity3{0.0, 0.0, 1.0}};

    const auto state2 = NavState{Pose3{Rot3::Yaw(0), Point3{0.0, 0.0, 2.0}}, Velocity3{0.0, 0.0, 1.0}};

    const auto noise_model = noiseModel::Constrained::All(3, mu);

    const auto factor = ConstantVelocityFactor(x1, x2, dt, noise_model);

    // positions are the same, secondary state has velocity 1.0 ,
    EXPECT(assert_equal(Unit3{0.0, 0.0, -1.0}.unitVector(), factor.evaluateError(origin, state0), tol));

    // same velocities, different position
    // second state agrees with initial state + velocity * dt
    EXPECT(assert_equal(Unit3{0.0, 0.0, 0.0}.unitVector(), factor.evaluateError(state0, state1), tol));

    // both bodies have the same velocity,
    // but state2.pose() != state0.pose() + state0.velocity * dt
    // error comes from this pose difference
    EXPECT(assert_equal(Unit3{0.0, 0.0, 1.0}.unitVector(), factor.evaluateError(state0, state2), tol));
}

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
