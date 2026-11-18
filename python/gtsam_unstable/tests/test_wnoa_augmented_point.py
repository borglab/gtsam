"""Python regression tests for WNOA with unstable AugmentedPoint3."""

import os
import sys
import unittest
from pathlib import Path

# Use the local build whether run from the source tree or by CMake's test target.
_test_path = Path(__file__).resolve()
_build_candidates = (
    _test_path.parents[2],  # build-x64/python when copied to the build tree
    _test_path.parents[3] / 'build-x64' / 'python',  # source-tree tests
)
_dll_handles = []
for _build_python in _build_candidates:
    if any((_build_python / 'gtsam').glob('*.pyd')) or any(
            (_build_python / 'gtsam').glob('*.so')):
        sys.path.insert(0, str(_build_python))
        if os.name == 'nt':
            _dll_handles = [os.add_dll_directory(str(_build_python / package))
                            for package in ('gtsam', 'gtsam_unstable')]
        break

import numpy as np
import gtsam
import gtsam_unstable as gu


def pose_key(index):
    return gtsam.symbol('p', index)


def velocity_key(index):
    return gtsam.symbol('v', index)


def insert_state(values, index, position, velocity):
    # AugmentedPoint3 is defined in unstable, so use its typed Values helper.
    gu.insertAugmentedPoint3(pose_key(index), gu.AugmentedPoint3(np.asarray(position)), values)
    values.insert(velocity_key(index), np.asarray(velocity, dtype=float))


class TestWnoaAugmentedPoint(unittest.TestCase):
    def test_motion_constant_velocity(self):
        left = gtsam.StateData(pose_key(0), velocity_key(0), 0.0)
        right = gtsam.StateData(pose_key(1), velocity_key(1), 0.2)
        velocity = np.array([0.5, -0.25, 0.75])
        position = np.array([1.0, -2.0, 3.0])
        factor = gu.WnoaMotionFactorAugmentedPoint3(
            left, right, np.ones(3)
        )

        values = gtsam.Values()
        insert_state(values, 0, position, velocity)
        insert_state(values, 1, position + 0.2 * velocity, velocity)
        self.assertAlmostEqual(factor.error(values), 0.0, places=9)

    def test_interpolated_squared_range(self):
        left = gtsam.StateData(pose_key(0), velocity_key(0), 0.0)
        right = gtsam.StateData(pose_key(1), velocity_key(1), 0.2)
        middle = gtsam.StateData(pose_key(2), velocity_key(2), 0.08)
        noise = gtsam.noiseModel.Isotropic.Sigma(1, 0.1)
        inner = gu.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
            middle.pose, np.zeros(3), 1.0, noise
        )
        factor = gu.WnoaInterpFactorAugmentedPoint3(
            inner, {left, right}, {middle}, np.ones(3), True, True
        )

        values = gtsam.Values()
        insert_state(values, 0, [1.0, 0.0, 0.0], [1.0, 0.0, 0.0])
        insert_state(values, 1, [1.2, 0.0, 0.0], [1.0, 0.0, 0.0])
        self.assertEqual(set(factor.keys()),
                         {left.pose, left.velocity, right.pose, right.velocity})
        self.assertEqual(factor.dim(), 1)
        residual = 1.08**2 - 1.0
        self.assertAlmostEqual(factor.error(values),
                               0.5 * (residual / 0.1)**2, places=7)

    def test_range_only_optimization(self):
        p0 = np.array([0.6, -0.2, 0.7])
        velocity = np.array([0.35, 0.1, -0.15])
        q = np.full(3, 0.2)
        knots = [gtsam.StateData(pose_key(i), velocity_key(i), float(i))
                 for i in range(3)]
        landmarks = [np.array(p, dtype=float) for p in (
            (0, 0, 0), (3, 0, 0), (0, 3, 0), (0, 0, 3)
        )]

        graph = gtsam.NonlinearFactorGraph()
        for left, right in zip(knots[:-1], knots[1:]):
            graph.add(gu.WnoaMotionFactorAugmentedPoint3(left, right, q))

        range_noise = gtsam.noiseModel.Isotropic.Sigma(1, 0.02)
        # Ranges at the first knot anchor the trajectory without requiring
        # an AugmentedPoint prior-factor binding.
        for landmark in landmarks:
            measured = np.linalg.norm(p0 - landmark)
            graph.add(gu.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
                knots[0].pose, landmark, measured, range_noise
            ))

        for i, t in enumerate((0.25, 0.75, 1.25, 1.75)):
            at_time = gtsam.StateData(pose_key(10 + i),
                                      velocity_key(10 + i), t)
            position = p0 + t * velocity
            for landmark in landmarks:
                measured = np.linalg.norm(position - landmark)
                inner = gu.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
                    at_time.pose, landmark, measured, range_noise
                )
                graph.add(gu.WnoaInterpFactorAugmentedPoint3(
                    inner, set(knots), {at_time}, q, True, True
                ))

        initial = gtsam.Values()
        for i in range(3):
            insert_state(initial, i,
                         p0 + i * velocity + np.array([0.12, -0.08, 0.10]),
                         velocity + np.array([-0.08, 0.06, 0.10]))

        initial_error = graph.error(initial)
        result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
        final_error = graph.error(result)

        self.assertGreater(initial_error, 0.0)
        self.assertLess(final_error, initial_error)
        self.assertLess(final_error, 1e-4)
        self.assertEqual(result.size(), 6)
        for i in range(3):
            position = np.asarray(gu.atAugmentedPoint3(
                pose_key(i), result).vector()).reshape(3)
            actual_velocity = np.asarray(result.atVector(velocity_key(i)))
            np.testing.assert_allclose(position, p0 + i * velocity, atol=1e-2)
            np.testing.assert_allclose(actual_velocity, velocity, atol=1e-2)


def lifted_state(values, index, position, velocity):
    """Insert the exact QCQP columns [h, x, u] and an unlifted 3x1 velocity."""
    gu.insertQcqpValue3(pose_key(index), gu.AugmentedPoint3(np.asarray(position)), values)
    values.insert(velocity_key(index), np.asarray(velocity, dtype=float).reshape(3, 1))


class TestWnoaQcqp(unittest.TestCase):
    """QCQP conversion of the AugmentedPoint3 WNOA factors."""

    Q = np.full(3, 0.2)
    LANDMARK = np.array([2.0, 0.0, 0.0])

    def test_motion_qcqp_matches_nonlinear_cost(self):
        left = gtsam.StateData(pose_key(0), velocity_key(0), 0.0)
        right = gtsam.StateData(pose_key(1), velocity_key(1), 0.3)
        graph = gtsam.NonlinearFactorGraph()
        graph.add(gu.WnoaMotionFactorAugmentedPoint3(left, right, self.Q))
        problem = gtsam.QcqpProblem(graph)

        # Two lift equalities per AugmentedPoint, no inequalities.
        _, equality_dim, inequality_dim = problem.dim()
        self.assertEqual((equality_dim, inequality_dim), (4, 0))

        typed, lifted = gtsam.Values(), gtsam.Values()
        states = [([0.1, -0.2, 0.3], [0.5, 0.1, -0.4]), ([0.4, 0.0, 0.1], [0.3, -0.2, 0.2])]
        for index, (position, velocity) in enumerate(states):
            insert_state(typed, index, position, velocity)
            lifted_state(lifted, index, position, velocity)
        cost, equality_violation, _ = problem.evaluate(lifted)
        self.assertGreater(cost, 0.0)
        self.assertAlmostEqual(cost, graph.error(typed), places=9)
        self.assertAlmostEqual(equality_violation, 0.0, places=12)

        # A lifted norm that disagrees with the coordinates violates a constraint.
        broken = gtsam.Values()
        lifted_state(broken, 1, *states[1])
        broken.insert(pose_key(0), np.array([[1.0], [0.1], [-0.2], [0.3], [5.0]]))
        broken.insert(velocity_key(0), np.array(states[0][1]).reshape(3, 1))
        self.assertGreater(problem.evaluate(broken)[1], 1.0)

    def test_unsupported_configurations_are_rejected(self):
        left = gtsam.StateData(pose_key(0), velocity_key(0), 0.0)
        right = gtsam.StateData(pose_key(1), velocity_key(1), 1.0)
        middle = gtsam.StateData(pose_key(2), velocity_key(2), 0.4)
        # Interpolated factors have no QCQP conversion of their own.
        inner = gu.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
            middle.pose, self.LANDMARK, 1.5, gtsam.noiseModel.Isotropic.Sigma(1, 0.1))
        interpolated = gtsam.NonlinearFactorGraph()
        interpolated.add(gu.WnoaInterpFactorAugmentedPoint3(
            inner, {left, right}, {middle}, self.Q, True, True))
        with self.assertRaises(RuntimeError):
            gtsam.QcqpProblem(interpolated)
        motion = gtsam.NonlinearFactorGraph()
        motion.add(gu.WnoaMotionFactorAugmentedPoint3(left, right, self.Q))
        with self.assertRaises(ValueError):
            gtsam.QcqpProblem(motion, 2)


def fix_homogeneous_coordinate(problem, keys):
    """Pin h = 1 on every lifted position, removing the per-point sign gauge."""
    h = np.array([[1.0, 0.0, 0.0, 0.0, 0.0]])
    for key in keys:
        problem.addConstraint(gtsam.LinearConstraint.Equal(
            gtsam.JacobianFactor(key, h, np.ones(1), gtsam.noiseModel.Unit.Create(1))))


class TestWnoaFullGp(unittest.TestCase):
    """Interpolated and full Gaussian process graphs from one measurement graph."""

    Q = np.array([0.2, 0.3, 0.25])
    LANDMARKS = [np.array(p, dtype=float) for p in ((0, 0, 0), (3, 0, 0), (0, 3, 0), (0, 0, 3))]
    KNOTS = {gtsam.StateData(pose_key(i), velocity_key(i), float(i)) for i in range(3)}
    MEASUREMENT_STATES = {gtsam.StateData(pose_key(10 + i), velocity_key(10 + i), t)
                          for i, t in enumerate((0.25, 0.5, 0.75, 1.4))}
    ALL_STATES = KNOTS | MEASUREMENT_STATES

    @staticmethod
    def true_position(t):
        return np.array([0.6 + 0.35 * t, -0.2 + 0.1 * t * t, 0.7 - 0.15 * t])

    def measurement_graph(self):
        """Squared ranges from the first knot and each measurement-time state, on their own keys."""
        graph = gtsam.NonlinearFactorGraph()
        first_knot = min(self.KNOTS, key=lambda state: state.time)
        for state in [first_knot] + sorted(self.MEASUREMENT_STATES, key=lambda s: s.time):
            for i, landmark in enumerate(self.LANDMARKS):
                measured = np.linalg.norm(self.true_position(state.time) - landmark) + (0.01 if i % 2 else -0.01)
                graph.add(gu.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
                    state.pose, landmark, float(measured), gtsam.noiseModel.Isotropic.Sigma(1, 0.05)))
        return graph

    def knot_values(self):
        values = gtsam.Values()
        for state in self.KNOTS:
            gu.insertAugmentedPoint3(state.pose, gu.AugmentedPoint3(
                self.true_position(state.time) + np.array([0.1, -0.05, 0.08])), values)
            values.insert(state.velocity, np.array([0.3, 0.2, -0.1]))
        return values

    def graphs(self):
        measurements = self.measurement_graph()
        interpolated = gu.interpolateFactorGraph3(
            measurements, self.KNOTS, self.MEASUREMENT_STATES, self.Q, True)
        full_gp = gu.interpolateFactorGraph3(measurements, self.ALL_STATES, set(), self.Q)
        return interpolated, full_gp

    def lift(self, values):
        lifted = gtsam.Values()
        for state in self.ALL_STATES:
            gu.insertQcqpValue3(state.pose, gu.atAugmentedPoint3(state.pose, values), lifted)
            # atPoint3 reads both Python-inserted and C++-inserted (Vector3) velocities.
            lifted.insert(state.velocity, values.atPoint3(state.velocity).reshape(3, 1))
        return lifted

    def test_state_partitions(self):
        interpolated, full_gp = self.graphs()
        self.assertEqual(interpolated.size(), 2 + 16 + 4)
        self.assertEqual(interpolated.keys().size(), 6)
        self.assertEqual(full_gp.size(), 6 + 20)
        self.assertEqual(full_gp.keys().size(), 14)

    def test_conditional_mean_matches_fixed_noise_interpolation(self):
        interpolated, full_gp = self.graphs()
        knots = self.knot_values()
        conditional_mean = gu.updateInterpValues3(
            interpolated, knots, self.KNOTS, self.MEASUREMENT_STATES, self.Q)
        self.assertGreater(interpolated.error(knots), 1.0)
        self.assertAlmostEqual(full_gp.error(conditional_mean) / interpolated.error(knots), 1.0, places=10)

    def test_qcqp_matches_nonlinear_with_unary_constraints(self):
        _, full_gp = self.graphs()
        problem = gtsam.QcqpProblem(full_gp)
        # Only the two lift equalities of each AugmentedPoint: no linear coupling rows.
        self.assertEqual(problem.dim()[1:], (2 * len(self.ALL_STATES), 0))
        values = gtsam.Values()
        for state in self.ALL_STATES:
            gu.insertAugmentedPoint3(state.pose, gu.AugmentedPoint3(
                self.true_position(state.time) + np.array([0.05 * state.time, -0.04, 0.03])), values)
            values.insert(state.velocity, np.array([0.3, 0.1 * state.time, -0.2]))
        cost, equality_violation, _ = problem.evaluate(self.lift(values))
        self.assertGreater(cost, 1.0)
        self.assertAlmostEqual(cost / full_gp.error(values), 1.0, places=10)
        self.assertAlmostEqual(equality_violation, 0.0, places=12)

    def test_augmented_lagrangian_matches_levenberg_marquardt(self):
        interpolated, full_gp = self.graphs()
        initial = gu.updateInterpValues3(
            interpolated, self.knot_values(), self.KNOTS, self.MEASUREMENT_STATES, self.Q)
        nonlinear = gtsam.LevenbergMarquardtOptimizer(full_gp, initial).optimize()

        problem = gtsam.QcqpProblem(full_gp)
        fix_homogeneous_coordinate(problem, [state.pose for state in self.ALL_STATES])
        params = gtsam.AugmentedLagrangianParams()
        params.updatePolicy = gtsam.AugmentedLagrangianUpdatePolicy.Aggressive
        params.maxIterations = 200
        params.absoluteViolationTolerance = 1e-9
        params.relativeViolationTolerance = 1e-9
        params.relativeCostTolerance = 1e-12
        params.absoluteStationarityTolerance = 1e-8
        solved = gtsam.AugmentedLagrangianOptimizer(problem, self.lift(initial), params).optimize()

        cost, equality_violation, _ = problem.evaluate(solved)
        self.assertAlmostEqual(equality_violation, 0.0, places=6)
        self.assertAlmostEqual(cost, full_gp.error(nonlinear), places=6)
        for state in self.KNOTS:
            np.testing.assert_allclose(
                gu.fromQcqpValue3(solved.atMatrix(state.pose)).vector().reshape(3),
                gu.atAugmentedPoint3(state.pose, nonlinear).vector().reshape(3), atol=1e-4)
            np.testing.assert_allclose(
                # A (3, 1) array inserted from Python is stored as a Vector.
                solved.atVector(state.velocity),
                nonlinear.atPoint3(state.velocity), atol=1e-4)

    @unittest.skipUnless(hasattr(gtsam, "MosekMonolithicSDP"), "GTSAM was built without MOSEK support")
    def test_sdp_accepts_full_gp_qcqp(self):
        interpolated, full_gp = self.graphs()
        initial = gu.updateInterpValues3(
            interpolated, self.knot_values(), self.KNOTS, self.MEASUREMENT_STATES, self.Q)
        nonlinear = gtsam.LevenbergMarquardtOptimizer(full_gp, initial).optimize()
        problem = gtsam.QcqpProblem(full_gp)
        fix_homogeneous_coordinate(problem, [state.pose for state in self.ALL_STATES])
        # The relaxation's optimum is a lower bound on any feasible cost; tightness is not asserted.
        for sdp in (gtsam.MosekMonolithicSDP(problem),
                    gtsam.MosekChordalSDP(problem, gtsam.ChordalOrderingType.Colamd)):
            sdp.solve()
            self.assertLessEqual(sdp.objectiveValue(), full_gp.error(nonlinear) * (1 + 1e-6) + 1e-9)


if __name__ == '__main__':
    unittest.main()
