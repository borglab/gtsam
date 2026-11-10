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


if __name__ == '__main__':
    unittest.main()
