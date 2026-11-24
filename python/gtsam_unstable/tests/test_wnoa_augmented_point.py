"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Tests for WNOA motion and interpolation factors with unstable AugmentedPoint
positions.
"""

import unittest

import gtsam
import gtsam_unstable
import numpy as np

T0, T1 = 1.0, 1.4  # Knot times of the test interval.
DT = T1 - T0
PSD = np.array([0.2, 0.5, 1.5])
P0, V0 = np.array([1.0, -2.0, 3.0]), np.array([0.5, -0.25, 0.75])
P1, V1 = np.array([1.3, -1.7, 2.8]), np.array([0.25, 0.1, 0.5])
T2 = 2.0  # Third knot for graph-level tests with two intervals.
P2, V2 = np.array([1.5, -1.6, 3.1]), np.array([0.4, 0.3, 0.2])


def pose_key(index):
    return gtsam.symbol("p", index)


def velocity_key(index):
    return gtsam.symbol("v", index)


def insert_state(values, state, position, velocity):
    """Insert an AugmentedPoint position and a velocity vector for a StateData."""
    dimension = len(position)
    point = getattr(gtsam_unstable, f"AugmentedPoint{dimension}")(np.asarray(position, dtype=float))
    getattr(gtsam_unstable, f"insertAugmentedPoint{dimension}")(state.pose, point, values)
    values.insert(state.velocity, np.asarray(velocity, dtype=float))


def conditional_mean(s, p0, v0, p1, v1, dt=DT):
    """WNOA conditional mean of a vector space: cubic Hermite interpolation at offset s."""
    u = s / dt
    position = ((2 * u**3 - 3 * u**2 + 1) * p0 + (u**3 - 2 * u**2 + u) * dt * v0
                + (-2 * u**3 + 3 * u**2) * p1 + (u**3 - u**2) * dt * v1)
    velocity = ((6 * u**2 - 6 * u) / dt * p0 + (3 * u**2 - 4 * u + 1) * v0
                + (-6 * u**2 + 6 * u) / dt * p1 + (3 * u**2 - 2 * u) * v1)
    return position, velocity


class TestWnoaAugmentedPoint(unittest.TestCase):
    """Nonlinear WNOA workflow with AugmentedPoint2 and AugmentedPoint3 positions."""

    def setUp(self):
        self.left = gtsam.StateData(pose_key(0), velocity_key(0), T0)
        self.right = gtsam.StateData(pose_key(1), velocity_key(1), T1)

    def knot_values(self, dimension=3):
        values = gtsam.Values()
        insert_state(values, self.left, P0[:dimension], V0[:dimension])
        insert_state(values, self.right, P1[:dimension], V1[:dimension])
        return values

    def three_knots(self, dimension=3):
        """Knots at T0, T1 and T2 with their values, for graph-level tests."""
        last = gtsam.StateData(pose_key(3), velocity_key(3), T2)
        values = self.knot_values(dimension)
        insert_state(values, last, P2[:dimension], V2[:dimension])
        return {self.left, self.right, last}, last, values

    def interpolated(self, inner, middle, dimension=3, fixed_noise=True):
        factor_type = getattr(gtsam_unstable, f"WnoaInterpFactorAugmentedPoint{dimension}")
        return factor_type(inner, {self.left, self.right}, {middle}, PSD[:dimension], fixed_noise, True)

    def test_motion_residual(self):
        """Constant velocity has zero residual; perturbations give [dp; dv] and 1/2 r'Q^-1 r."""
        for dimension in (2, 3):
            factor_type = getattr(gtsam_unstable, f"WnoaMotionFactorAugmentedPoint{dimension}")
            factor = factor_type(self.left, self.right, PSD[:dimension])
            self.assertEqual(list(factor.keys()), [self.left.pose, self.left.velocity,
                                                   self.right.pose, self.right.velocity])
            p0, v0 = P0[:dimension], V0[:dimension]

            values = gtsam.Values()
            insert_state(values, self.left, p0, v0)
            insert_state(values, self.right, p0 + DT * v0, v0)
            self.assertAlmostEqual(factor.error(values), 0.0, places=12)

            dp, dv = np.array([0.03, -0.02, 0.05])[:dimension], np.array([-0.1, 0.2, 0.05])[:dimension]
            values = gtsam.Values()
            insert_state(values, self.left, p0, v0)
            insert_state(values, self.right, p0 + DT * v0 + dp, v0 + dv)
            residual = np.concatenate([dp, dv])
            np.testing.assert_allclose(factor.unwhitenedError(values), residual, atol=1e-12)
            Qc = np.diag(PSD[:dimension])
            Q = np.block([[DT**3 / 3 * Qc, DT**2 / 2 * Qc], [DT**2 / 2 * Qc, DT * Qc]])
            self.assertAlmostEqual(factor.error(values) / (0.5 * residual @ np.linalg.solve(Q, residual)),
                                   1.0, places=10)

    def test_interpolated_position_and_velocity(self):
        """Interpolated states equal the WNOA conditional mean.

        The position is recovered from squared ranges with zero measured range: to
        the origin, |q|^2, and to each unit axis, |q|^2 - 2 q_i + 1. The velocity is
        read through a Point3 prior on the interpolated velocity key.
        """
        s = 0.13
        noise = gtsam.noiseModel.Isotropic.Sigma(1, 0.1)
        for dimension in (2, 3):
            middle = gtsam.StateData(pose_key(2), velocity_key(2), T0 + s)
            range_type = getattr(gtsam_unstable, f"KnownLandmarkSquaredRangeFactorAugmentedPoint{dimension}")
            values = self.knot_values(dimension)

            def squared_range(landmark):
                factor = self.interpolated(range_type(middle.pose, landmark, 0.0, noise), middle, dimension)
                return factor.unwhitenedError(values)[0]

            origin = squared_range(np.zeros(dimension))
            position = np.array([(origin - squared_range(np.eye(dimension)[i]) + 1.0) / 2.0
                                 for i in range(dimension)])
            expected_position, expected_velocity = conditional_mean(
                s, P0[:dimension], V0[:dimension], P1[:dimension], V1[:dimension])
            np.testing.assert_allclose(position, expected_position, atol=1e-12)

            if dimension == 3:
                prior = gtsam.PriorFactorPoint3(middle.velocity, np.zeros(3),
                                                gtsam.noiseModel.Isotropic.Sigma(3, 1.0))
                velocity = self.interpolated(prior, middle).unwhitenedError(values)
                np.testing.assert_allclose(velocity, expected_velocity, atol=1e-12)

    def test_interpolated_squared_range(self):
        """Residual, error and keys of an interpolated squared range in both noise modes."""
        middle = gtsam.StateData(pose_key(2), velocity_key(2), T0 + 0.13)
        landmark, measured, sigma = np.array([2.0, 0.5, -1.0]), 2.5, 0.1
        inner = gtsam_unstable.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
            middle.pose, landmark, measured, gtsam.noiseModel.Isotropic.Sigma(1, sigma))
        fixed = self.interpolated(inner, middle, fixed_noise=True)
        inflated = self.interpolated(inner, middle, fixed_noise=False)
        values = self.knot_values()

        self.assertEqual(set(fixed.keys()), {self.left.pose, self.left.velocity,
                                             self.right.pose, self.right.velocity})
        self.assertEqual(fixed.dim(), 1)
        position, _ = conditional_mean(0.13, P0, V0, P1, V1)
        residual = np.sum((position - landmark)**2) - measured**2
        for factor in (fixed, inflated):
            self.assertAlmostEqual(factor.unwhitenedError(values)[0], residual, places=12)
        self.assertAlmostEqual(fixed.error(values), 0.5 * (residual / sigma)**2, places=9)
        # Interpolation noise inflates the measurement variance and lowers the error.
        self.assertLess(inflated.error(values), fixed.error(values))

    def test_rejects_times_outside_knots(self):
        """Interpolated times outside the knots are rejected rather than extrapolated."""
        noise = gtsam.noiseModel.Isotropic.Sigma(1, 0.1)
        for t in (T0 - 0.1, T1 + 0.1):
            outside = gtsam.StateData(pose_key(3), velocity_key(3), t)
            inner = gtsam_unstable.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
                outside.pose, np.zeros(3), 1.0, noise)
            with self.assertRaises(RuntimeError):
                self.interpolated(inner, outside)

    def test_range_only_optimization(self):
        """Ranges at intermediate times recover a constant-velocity trajectory."""
        p0, velocity = np.array([0.6, -0.2, 0.7]), np.array([0.35, 0.1, -0.15])
        psd = np.full(3, 0.2)
        knots = [gtsam.StateData(pose_key(i), velocity_key(i), float(i)) for i in range(3)]
        landmarks = [np.array(p, dtype=float) for p in ((0, 0, 0), (3, 0, 0), (0, 3, 0), (0, 0, 3))]
        noise = gtsam.noiseModel.Isotropic.Sigma(1, 0.02)

        graph = gtsam.NonlinearFactorGraph()
        for left, right in zip(knots[:-1], knots[1:]):
            graph.add(gtsam_unstable.WnoaMotionFactorAugmentedPoint3(left, right, psd))
        # Ranges at the first knot anchor the trajectory.
        for landmark in landmarks:
            graph.add(gtsam_unstable.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
                knots[0].pose, landmark, float(np.linalg.norm(p0 - landmark)), noise))
        # Ranges between the knots act on interpolated positions.
        for i, t in enumerate((0.25, 0.75, 1.25, 1.75)):
            at_time = gtsam.StateData(pose_key(10 + i), velocity_key(10 + i), t)
            for landmark in landmarks:
                inner = gtsam_unstable.KnownLandmarkSquaredRangeFactorAugmentedPoint3(
                    at_time.pose, landmark, float(np.linalg.norm(p0 + t * velocity - landmark)), noise)
                graph.add(gtsam_unstable.WnoaInterpFactorAugmentedPoint3(
                    inner, set(knots), {at_time}, psd, True, True))

        initial = gtsam.Values()
        for i, state in enumerate(knots):
            insert_state(initial, state, p0 + i * velocity + np.array([0.12, -0.08, 0.10]),
                         velocity + np.array([-0.08, 0.06, 0.10]))
        initial_error = graph.error(initial)
        result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()

        self.assertGreater(initial_error, 1.0)
        self.assertLess(graph.error(result), 1e-8)
        self.assertEqual(result.size(), 6)  # No states at the measurement times.
        for i, state in enumerate(knots):
            np.testing.assert_allclose(
                gtsam_unstable.atAugmentedPoint3(state.pose, result).vector().ravel(),
                p0 + i * velocity, atol=1e-4)
            np.testing.assert_allclose(result.atVector(state.velocity), velocity, atol=1e-4)

    def test_interpolate_factor_graph(self):
        """interpolateFactorGraph adds motion factors and wraps factors on interpolated states."""
        noise = gtsam.noiseModel.Isotropic.Sigma(1, 0.1)
        for dimension in (2, 3):
            psd = PSD[:dimension]
            motion_type = getattr(gtsam_unstable, f"WnoaMotionFactorAugmentedPoint{dimension}")
            interp_type = getattr(gtsam_unstable, f"WnoaInterpFactorAugmentedPoint{dimension}")
            range_type = getattr(gtsam_unstable, f"KnownLandmarkSquaredRangeFactorAugmentedPoint{dimension}")
            interpolate = getattr(gtsam_unstable, f"interpolateFactorGraphAugmentedPoint{dimension}")
            knots, last, values = self.three_knots(dimension)
            middle = gtsam.StateData(pose_key(10), velocity_key(10), T1 + 0.2)
            landmark = np.array([2.0, 0.5, -1.0])[:dimension]
            on_knot = range_type(self.left.pose, landmark, 2.5, noise)
            on_middle = range_type(middle.pose, landmark, 2.5, noise)

            graph = gtsam.NonlinearFactorGraph()
            graph.add(motion_type(self.left, self.right, psd))  # Replaced, not duplicated.
            graph.add(on_knot)
            graph.add(on_middle)

            for fixed_noise in (True, False):
                result = interpolate(graph, knots, {middle}, psd, fixed_noise)
                self.assertEqual(result.size(), 4)
                motions, knot_factor, wrapped = [result.at(0), result.at(1)], result.at(2), result.at(3)

                # One WNOA motion factor between each pair of neighbouring knots.
                for factor, (a, b) in zip(motions, [(self.left, self.right), (self.right, last)]):
                    self.assertIsInstance(factor, motion_type)
                    self.assertEqual(list(factor.keys()), [a.pose, a.velocity, b.pose, b.velocity])
                    self.assertAlmostEqual(factor.error(values), motion_type(a, b, psd).error(values),
                                           places=12)

                # Factors on knots are copied; factors on interpolated states are wrapped.
                self.assertEqual(list(knot_factor.keys()), [self.left.pose])
                self.assertAlmostEqual(knot_factor.error(values), on_knot.error(values), places=12)
                self.assertIsInstance(wrapped, interp_type)
                self.assertEqual(set(wrapped.keys()),
                                 {self.right.pose, self.right.velocity, last.pose, last.velocity})
                expected = interp_type(on_middle, {self.right, last}, {middle}, psd, fixed_noise, True)
                self.assertAlmostEqual(wrapped.error(values), expected.error(values), places=12)

                # The interpolated state does not become a variable.
                self.assertNotIn(middle.pose, set(result.keyVector()))
                self.assertNotIn(middle.velocity, set(result.keyVector()))

            outside = gtsam.StateData(pose_key(11), velocity_key(11), T2 + 0.1)
            with self.assertRaises(RuntimeError):
                interpolate(graph, knots, {outside}, psd)

    def test_update_interp_values(self):
        """updateInterpValues adds interpolated states and keeps the knot values."""
        noise = gtsam.noiseModel.Isotropic.Sigma(1, 0.1)
        for dimension in (2, 3):
            psd = PSD[:dimension]
            range_type = getattr(gtsam_unstable, f"KnownLandmarkSquaredRangeFactorAugmentedPoint{dimension}")
            interpolate = getattr(gtsam_unstable, f"interpolateFactorGraphAugmentedPoint{dimension}")
            update = getattr(gtsam_unstable, f"updateInterpValuesAugmentedPoint{dimension}")
            at_position = getattr(gtsam_unstable, f"atAugmentedPoint{dimension}")
            at_velocity = getattr(gtsam.Values, f"atPoint{dimension}")  # Fixed-size tangent vector.
            knots, _, values = self.three_knots(dimension)
            middles = {gtsam.StateData(pose_key(10 + i), velocity_key(10 + i), t)
                       for i, t in enumerate((T0 + 0.1, T1 + 0.3))}
            landmark = np.array([2.0, 0.5, -1.0])[:dimension]

            graph = gtsam.NonlinearFactorGraph()
            inner_factors = [range_type(m.pose, landmark, 2.5, noise) for m in middles]
            for factor in inner_factors:
                graph.add(factor)
            interp_graph = interpolate(graph, knots, middles, psd, True)
            updated = update(interp_graph, values, knots, middles, psd)

            self.assertEqual(updated.size(), values.size() + 2 * len(middles))
            for knot in knots:
                np.testing.assert_array_equal(at_position(knot.pose, updated).vector(),
                                              at_position(knot.pose, values).vector())
            for middle in middles:
                self.assertTrue(updated.exists(middle.pose) and updated.exists(middle.velocity))
                self.assertEqual(at_velocity(updated, middle.velocity).shape, (dimension,))

            # Inner factors on the inserted states agree with the wrapped factors on the knots.
            wrapped = [interp_graph.at(i) for i in range(2, interp_graph.size())]
            for inner, outer in zip(inner_factors, wrapped):
                np.testing.assert_allclose(inner.unwhitenedError(updated),
                                           outer.unwhitenedError(values), atol=1e-12)

    def test_update_interp_values_matches_conditional_mean(self):
        """Interpolated positions and velocities equal the WNOA conditional mean."""
        intervals = [(T0, T1, P0, V0, P1, V1), (T1, T2, P1, V1, P2, V2)]
        offsets = (0.05, 0.13, 0.31)
        for dimension in (2, 3):
            psd = PSD[:dimension]
            update = getattr(gtsam_unstable, f"updateInterpValuesAugmentedPoint{dimension}")
            update_point = getattr(gtsam, f"updateInterpValuesPoint{dimension}")
            at_position = getattr(gtsam_unstable, f"atAugmentedPoint{dimension}")
            at_point = getattr(gtsam.Values, f"atPoint{dimension}")
            knots, _, values = self.three_knots(dimension)

            # The same knots as ordinary points, for the stable Point instantiation.
            point_values = gtsam.Values()
            for knot in knots:
                point_values.insert(knot.pose, at_position(knot.pose, values).vector().ravel())
                point_values.insert(knot.velocity, values.atVector(knot.velocity))

            expected, middles = {}, set()
            for k, (ta, tb, pa, va, pb, vb) in enumerate(intervals):
                for j, s in enumerate(offsets):
                    middle = gtsam.StateData(pose_key(10 + 10 * k + j), velocity_key(10 + 10 * k + j), ta + s)
                    middles.add(middle)
                    expected[middle.pose, middle.velocity] = conditional_mean(
                        s, pa[:dimension], va[:dimension], pb[:dimension], vb[:dimension], tb - ta)

            graph = gtsam.NonlinearFactorGraph()
            updated = update(graph, values, knots, middles, psd)
            point_updated = update_point(graph, point_values, knots, middles, psd)
            for (pose, velocity), (position, velocity_mean) in expected.items():
                np.testing.assert_allclose(at_position(pose, updated).vector().ravel(), position, atol=1e-12)
                np.testing.assert_allclose(at_point(updated, velocity), velocity_mean, atol=1e-12)
                np.testing.assert_allclose(at_position(pose, updated).vector().ravel(),
                                           at_point(point_updated, pose), atol=1e-12)
                np.testing.assert_allclose(at_point(updated, velocity),
                                           at_point(point_updated, velocity), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
