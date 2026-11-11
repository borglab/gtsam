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


def fix_homogeneous_coordinate(problem, keys):
    """Pin h = 1 on every lifted position, removing the per-point sign gauge."""
    h = np.array([[1.0, 0.0, 0.0, 0.0, 0.0]])
    for key in keys:
        problem.addConstraint(gtsam.LinearConstraint.Equal(
            gtsam.JacobianFactor(key, h, np.ones(1), gtsam.noiseModel.Unit.Create(1))))


class TestWnoaQcqp(unittest.TestCase):
    """QCQP conversion of the AugmentedPoint3 WNOA factors."""

    Q = np.full(3, 0.2)
    LANDMARK = np.array([2.0, 0.0, 0.0])

    def interpolated_graph(self, ranges=(1.5,), fixed_noise=True):
        left = gtsam.StateData(pose_key(0), velocity_key(0), 0.0)
        right = gtsam.StateData(pose_key(1), velocity_key(1), 1.0)
        middle = gtsam.StateData(pose_key(2), velocity_key(2), 0.4)
        graph = gtsam.NonlinearFactorGraph()
        for measured in ranges:
            inner = gu.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
                middle.pose, self.LANDMARK, measured,
                gtsam.noiseModel.Isotropic.Sigma(1, 0.1))
            graph.add(gu.WnoaInterpFactorAugmentedPoint3(
                inner, {left, right}, {middle}, self.Q, fixed_noise, True))
        return graph

    def constant_velocity_values(self, position, velocity):
        """Typed knot values and lifted values (with the interpolated point at t=0.4)."""
        typed, lifted = gtsam.Values(), gtsam.Values()
        for index in (0, 1):
            insert_state(typed, index, position + index * velocity, velocity)
            lifted_state(lifted, index, position + index * velocity, velocity)
        gu.insertQcqpValue3(pose_key(2), gu.AugmentedPoint3(position + 0.4 * velocity), lifted)
        return typed, lifted

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

    def test_interpolated_range_qcqp_matches_nonlinear_cost(self):
        graph = self.interpolated_graph()
        problem = gtsam.QcqpProblem(graph)
        # Lift constraints for two knots and the interpolated point (6 rows) plus
        # the 3-row linear link of the interpolated coordinates.
        self.assertEqual(problem.dim()[1:], (9, 0))

        position = np.array([0.5, -0.1, 0.2])
        velocity = np.array([0.4, 0.2, -0.1])
        typed, lifted = self.constant_velocity_values(position, velocity)
        cost, equality_violation, _ = problem.evaluate(lifted)
        residual = np.linalg.norm(position + 0.4 * velocity - self.LANDMARK)**2 - 1.5**2
        self.assertAlmostEqual(cost, 0.5 * (residual / 0.1)**2, places=7)
        self.assertAlmostEqual(cost, graph.error(typed), places=7)
        self.assertAlmostEqual(equality_violation, 0.0, places=9)

        # Moving the interpolated point away from the knot interpolation breaks the link.
        moved = gtsam.Values()
        lifted_state(moved, 0, position, velocity)
        lifted_state(moved, 1, position + velocity, velocity)
        gu.insertQcqpValue3(pose_key(2), gu.AugmentedPoint3(position + 0.4 * velocity + 0.05), moved)
        self.assertGreater(problem.evaluate(moved)[1], 1e-3)

    def test_negated_lift_violates_coupling(self):
        # Negating a lifted point keeps its own lift constraints satisfied but
        # breaks the linear coupling, so every point needs the constraint h = 1.
        problem = gtsam.QcqpProblem(self.interpolated_graph())
        position = np.array([0.5, -0.1, 0.2])
        velocity = np.array([0.4, 0.2, -0.1])
        _, lifted = self.constant_velocity_values(position, velocity)
        self.assertAlmostEqual(problem.evaluate(lifted)[1], 0.0, places=9)

        flipped = gtsam.Values()
        lifted_state(flipped, 1, position + velocity, velocity)
        gu.insertQcqpValue3(pose_key(2), gu.AugmentedPoint3(position + 0.4 * velocity), flipped)
        flipped.insert(pose_key(0), -gu.qcqpValue3(gu.AugmentedPoint3(position)))
        flipped.insert(velocity_key(0), velocity)
        self.assertGreater(problem.evaluate(flipped)[1], 1e-3)

    def test_shared_interpolated_key_is_coupled_once(self):
        # Several ranges at one time share the interpolated point and its constraints.
        problem = gtsam.QcqpProblem(self.interpolated_graph(ranges=(1.5, 1.4, 1.6)))
        self.assertEqual(problem.dim()[1:], (9, 0))

    def test_unsupported_configurations_are_rejected(self):
        with self.assertRaises(RuntimeError):
            gtsam.QcqpProblem(self.interpolated_graph(fixed_noise=False))
        with self.assertRaises(ValueError):
            gtsam.QcqpProblem(self.interpolated_graph(), 2)
        left = gtsam.StateData(pose_key(0), velocity_key(0), 0.0)
        right = gtsam.StateData(pose_key(1), velocity_key(1), 1.0)
        motion = gtsam.NonlinearFactorGraph()
        motion.add(gu.WnoaMotionFactorAugmentedPoint3(left, right, self.Q))
        with self.assertRaises(ValueError):
            gtsam.QcqpProblem(motion, 2)

    def test_range_only_qcqp_matches_levenberg_marquardt(self):
        p0 = np.array([0.6, -0.2, 0.7])
        velocity = np.array([0.35, 0.1, -0.15])
        knots = [gtsam.StateData(pose_key(i), velocity_key(i), float(i)) for i in range(3)]
        landmarks = [np.array(p, dtype=float) for p in ((0, 0, 0), (3, 0, 0), (0, 3, 0), (0, 0, 3))]
        times = (0.25, 0.75, 1.25, 1.75)
        noise = gtsam.noiseModel.Isotropic.Sigma(1, 0.05)

        graph = gtsam.NonlinearFactorGraph()
        for left, right in zip(knots[:-1], knots[1:]):
            graph.add(gu.WnoaMotionFactorAugmentedPoint3(left, right, self.Q))
        for i, t in enumerate(times):
            at_time = gtsam.StateData(pose_key(10 + i), velocity_key(10 + i), t)
            for landmark in landmarks:
                inner = gu.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
                    at_time.pose, landmark, float(np.linalg.norm(p0 + t * velocity - landmark)), noise)
                graph.add(gu.WnoaInterpFactorAugmentedPoint3(
                    inner, set(knots), {at_time}, self.Q, True, True))

        initial, lifted = gtsam.Values(), gtsam.Values()
        for i in range(3):
            position = p0 + i * velocity + np.array([0.05, -0.04, 0.03])
            guess = velocity + np.array([-0.04, 0.03, 0.05])
            insert_state(initial, i, position, guess)
            lifted_state(lifted, i, position, guess)
        for i, t in enumerate(times):
            k = int(t)
            gu.insertQcqpValue3(pose_key(10 + i), gu.AugmentedPoint3(
                p0 + k * velocity + np.array([0.05, -0.04, 0.03]) + (t - k) * velocity), lifted)
        nonlinear = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()

        problem = gtsam.QcqpProblem(graph)
        fix_homogeneous_coordinate(problem, [pose_key(i) for i in range(3)] + [pose_key(10 + i) for i in range(4)])
        params = gtsam.AugmentedLagrangianParams()
        params.updatePolicy = gtsam.AugmentedLagrangianUpdatePolicy.Aggressive
        params.maxIterations = 200
        params.absoluteViolationTolerance = 1e-9
        params.relativeViolationTolerance = 1e-9
        params.relativeCostTolerance = 1e-12
        params.absoluteStationarityTolerance = 1e-8
        solved = gtsam.AugmentedLagrangianOptimizer(problem, lifted, params).optimize()

        cost, equality_violation, _ = problem.evaluate(solved)
        self.assertAlmostEqual(equality_violation, 0.0, places=5)
        self.assertAlmostEqual(cost, graph.error(nonlinear), places=5)
        for i in range(3):
            np.testing.assert_allclose(
                gu.fromQcqpValue3(solved.atMatrix(pose_key(i))).vector().reshape(3),
                gu.atAugmentedPoint3(pose_key(i), nonlinear).vector().reshape(3), atol=1e-4)
            np.testing.assert_allclose(
                solved.atVector(velocity_key(i)), nonlinear.atVector(velocity_key(i)), atol=1e-4)


if __name__ == '__main__':
    unittest.main()
