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
 * @author  Asa Hammond
 */

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ConstantVelocityFactor.h>

#include <CppUnitLite/TestHarness.h>
#include <list>

/* ************************************************************************* */
TEST(ConstantVelocityFactor, VelocityFactor) {
    using namespace gtsam;

    const double tol{1e-5};

    const Key x1 = Key{1};
    const Key x2 = Key{2};

    const double dt{1.0};

    // moving upward with groundtruth velocity"
    const auto origin = NavState{Pose3{Rot3::Yaw(0), Point3{0.0, 0.0, 0.0}}, Velocity3{0.0, 0.0, 0.0}};

    const auto state0 = NavState{Pose3{Rot3::Yaw(0), Point3{0.0, 0.0, 0.0}}, Velocity3{0.0, 0.0, 1.0}};

    const auto state1 = NavState{Pose3{Rot3::Yaw(0), Point3{0.0, 0.0, 1.0}}, Velocity3{0.0, 0.0, 1.0}};

    const auto state2 = NavState{Pose3{Rot3::Yaw(0), Point3{0.0, 0.0, 2.0}}, Velocity3{0.0, 0.0, 1.0}};

    const auto state3 = NavState{Pose3{Rot3::Yaw(M_PI_2), Point3{0.0, 0.0, 2.0}}, Velocity3{0.0, 0.0, 1.0}};

    const double mu{1000};
    const auto noise_model = noiseModel::Constrained::All(9, mu);

    const auto factor = ConstantVelocityFactor(x1, x2, dt, noise_model);

    // positions are the same, secondary state has velocity 1.0 in z,
    const auto state0_err_origin = factor.evaluateError(origin, state0);
    EXPECT(assert_equal((Vector9() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished(), state0_err_origin, tol));

    // same velocities, different position
    // second state agrees with initial state + velocity * dt
    const auto state1_err_state0 = factor.evaluateError(state0, state1);
    EXPECT(assert_equal((Vector9() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished(), state1_err_state0, tol));

    // same velocities, same position, different rotations
    // second state agrees with initial state + velocity * dt
    // as we assume that omega is 0.0 this is the same as the above case
    //  TODO: this should respect omega and actually fail in this case
    const auto state3_err_state2 = factor.evaluateError(state0, state1);
    EXPECT(assert_equal((Vector9() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished(), state3_err_state2, tol));

    // both bodies have the same velocity,
    // but state2.pose() does not agree with state0.update()
    // error comes from this position difference
    const auto state2_err_state0 = factor.evaluateError(state0, state2);
    EXPECT(assert_equal((Vector9() << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished(), state2_err_state0, tol));
}

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
