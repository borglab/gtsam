"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Tests for the matrix-weighted localization dataset generator.
"""

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import gtsam
import numpy as np
from gtsam.examples.MatrixWeightedLocalizationExample import (
    MatrixWeightedLocalization,
)
from gtsam.symbol_shorthand import L


class TestMatrixWeightedLocalizationExample(unittest.TestCase):
    """Tests deterministic generation and g2o round-tripping."""

    def test_perturbed_measurements_are_deterministic(self):
        """Repeated seeded sampling produces identical measurements."""
        example = MatrixWeightedLocalization(3)
        first = example.perturbed_measurements()
        second = example.perturbed_measurements()

        self.assertEqual(len(first.landmarks), 12)
        self.assertEqual(len(first.odometry), 2)
        for actual, expected in zip(first.landmarks, second.landmarks):
            self.assertEqual((actual.k, actual.l), (expected.k, expected.l))
            np.testing.assert_allclose(actual.kP, expected.kP)
        for actual, expected in zip(first.odometry, second.odometry):
            self.assertTrue(actual.iTj.equals(expected.iTj, 1e-12))

    def test_g2o_round_trip(self):
        """Writing and reading preserves graph structure and Cartesian noise."""
        example = MatrixWeightedLocalization(3)
        measurements = example.perturbed_measurements()
        expected_landmarks = {
            (measurement.k, L(measurement.l)): measurement
            for measurement in measurements.landmarks
        }
        expected_odometry = {
            (measurement.i, measurement.j): measurement
            for measurement in measurements.odometry
        }

        with TemporaryDirectory() as directory:
            filename = str(Path(directory) / "matrix_weighted.g2o")
            example.write_g2o(filename, measurements)
            graph, values = gtsam.readG2o(filename, is3D=True)

        self.assertEqual(graph.size(), 14)
        self.assertEqual(values.size(), 7)
        for k, wTk in enumerate(example.wTks):
            self.assertTrue(values.atPose3(k).equals(wTk, 1e-5))
        for l, wL in enumerate(example.wLs):
            np.testing.assert_allclose(values.atPoint3(L(l)), wL)

        landmark_count = 0
        odometry_count = 0
        for index in range(graph.size()):
            factor = graph.at(index)
            keys = tuple(factor.keys())
            if isinstance(factor, gtsam.BearingRangeFactor3D):
                landmark_count += 1
                expected = expected_landmarks[keys]
                measured = factor.measured()
                range_ = measured.range()
                bearing = measured.bearing()
                J = np.column_stack((range_ * bearing.basis(), bearing.unitVector()))
                kP = range_ * bearing.unitVector()
                covariance = J @ factor.noiseModel().covariance() @ J.T
                np.testing.assert_allclose(kP, expected.kP, rtol=1e-10)
                np.testing.assert_allclose(
                    covariance,
                    np.linalg.inv(expected.information),
                    rtol=1e-9,
                    atol=1e-12,
                )
            elif isinstance(factor, gtsam.BetweenFactorPose3):
                odometry_count += 1
                expected = expected_odometry[keys]
                self.assertTrue(factor.measured().equals(expected.iTj, 1e-5))
            else:
                self.fail(f"Unexpected factor type: {type(factor).__name__}")

        self.assertEqual(landmark_count, 12)
        self.assertEqual(odometry_count, 2)


if __name__ == "__main__":
    unittest.main()
