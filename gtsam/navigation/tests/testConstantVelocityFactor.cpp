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

    // TODO make these tests way less verbose!
    // ideally I could find an initializer for Vector9 like: Vector9{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}'
    // positions are the same, secondary state has velocity 1.0 in z,
    const auto state0_err_origin = factor.evaluateError(origin, state0);
    EXPECT(assert_equal(0.0, NavState::dP(state0_err_origin).x(), tol));
    EXPECT(assert_equal(0.0, NavState::dP(state0_err_origin).y(), tol));
    EXPECT(assert_equal(0.0, NavState::dP(state0_err_origin).z(), tol));
    EXPECT(assert_equal(0.0, NavState::dR(state0_err_origin).x(), tol));
    EXPECT(assert_equal(0.0, NavState::dR(state0_err_origin).y(), tol));
    EXPECT(assert_equal(0.0, NavState::dR(state0_err_origin).z(), tol));
    EXPECT(assert_equal(0.0, NavState::dV(state0_err_origin).x(), tol));
    EXPECT(assert_equal(0.0, NavState::dV(state0_err_origin).y(), tol));
    EXPECT(assert_equal(1.0, NavState::dV(state0_err_origin).z(), tol));

    // same velocities, different position
    // second state agrees with initial state + velocity * dt
    const auto state1_err_state0 = factor.evaluateError(state0, state1);
    EXPECT(assert_equal(0.0, NavState::dP(state1_err_state0).x(), tol));
    EXPECT(assert_equal(0.0, NavState::dP(state1_err_state0).y(), tol));
    EXPECT(assert_equal(0.0, NavState::dP(state1_err_state0).z(), tol));
    EXPECT(assert_equal(0.0, NavState::dR(state1_err_state0).x(), tol));
    EXPECT(assert_equal(0.0, NavState::dR(state1_err_state0).y(), tol));
    EXPECT(assert_equal(0.0, NavState::dR(state1_err_state0).z(), tol));
    EXPECT(assert_equal(0.0, NavState::dV(state1_err_state0).x(), tol));
    EXPECT(assert_equal(0.0, NavState::dV(state1_err_state0).y(), tol));
    EXPECT(assert_equal(0.0, NavState::dV(state1_err_state0).z(), tol));

    // both bodies have the same velocity,
    // but state2.pose() does not agree with .update()
    // error comes from this pose difference
    const auto state2_err_state0 = factor.evaluateError(state0, state2);
    EXPECT(assert_equal(0.0, NavState::dP(state2_err_state0).x(), tol));
    EXPECT(assert_equal(0.0, NavState::dP(state2_err_state0).y(), tol));
    EXPECT(assert_equal(1.0, NavState::dP(state2_err_state0).z(), tol));
    EXPECT(assert_equal(0.0, NavState::dR(state2_err_state0).x(), tol));
    EXPECT(assert_equal(0.0, NavState::dR(state2_err_state0).y(), tol));
    EXPECT(assert_equal(0.0, NavState::dR(state2_err_state0).z(), tol));
    EXPECT(assert_equal(0.0, NavState::dV(state2_err_state0).x(), tol));
    EXPECT(assert_equal(0.0, NavState::dV(state2_err_state0).y(), tol));
    EXPECT(assert_equal(0.0, NavState::dV(state2_err_state0).z(), tol));
}

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
