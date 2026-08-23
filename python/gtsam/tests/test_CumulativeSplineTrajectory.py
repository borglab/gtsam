"""Tests for the generated cumulative spline trajectory wrappers."""

import unittest

import gtsam
import numpy as np


class TestCumulativeSplineTrajectory(unittest.TestCase):
    """Exercise value sampling through the production C++ implementation."""

    def test_cardinal_basis_weights(self):
        """The wrapped cardinal basis exposes the production cubic weights."""
        weights = gtsam.CardinalSplineBasis.CalculateWeights(4, 3.5)
        np.testing.assert_allclose(
            weights, [1.0 / 48.0, 23.0 / 48.0, 23.0 / 48.0, 1.0 / 48.0]
        )
        self.assertAlmostEqual(np.sum(weights), 1.0)

    def test_pose2_value_and_derivative(self):
        """The Pose2 wrapper samples constant controls and tangent rates."""
        trajectory = gtsam.CumulativeSplineTrajectoryPose2()
        for y in (0.0, 0.0, 2.0, 2.0):
            trajectory.addControlPoint(gtsam.Pose2(0.0, y, 0.0))

        actual = trajectory.sampleTrajectory(3.5)
        expected = gtsam.Pose2(0.0, 1.0, 0.0)
        self.assertTrue(actual.equals(expected, 1e-9))

        tangent_rate = trajectory.sampleTrajectoryDerivative(3.5)
        np.testing.assert_allclose(tangent_rate, [0.0, 1.5, 0.0], atol=1e-9)

    def test_wrapper_configuration(self):
        """The generated wrapper exposes density and front padding."""
        trajectory = gtsam.CumulativeSplineTrajectoryPose3(10.0, True)
        self.assertEqual(trajectory.density(), 10.0)
        self.assertTrue(trajectory.padFront())


if __name__ == "__main__":
    unittest.main()
