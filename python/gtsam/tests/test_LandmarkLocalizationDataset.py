"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Tests for the landmark localization dataset generator.
"""

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import gtsam
import numpy as np
from gtsam.examples.LandmarkLocalizationDataset import (
    LandmarkLocalizationDataset,
)
from gtsam.symbol_shorthand import L


class TestLandmarkLocalizationDataset(unittest.TestCase):
    """Tests deterministic generation and g2o round-tripping."""

    def test_perturbed_measurements_are_deterministic(self):
        """Repeated seeded sampling produces identical measurements."""
        dataset = LandmarkLocalizationDataset(3)
        first = dataset.perturbed_measurements()
        second = dataset.perturbed_measurements()

        self.assertEqual(len(first.landmarks), 12)
        self.assertEqual(len(first.odometry), 2)
        for actual, expected in zip(first.landmarks, second.landmarks):
            self.assertEqual((actual.k, actual.l), (expected.k, expected.l))
            np.testing.assert_allclose(actual.kP, expected.kP)
        for actual, expected in zip(first.odometry, second.odometry):
            self.assertTrue(actual.iTj.equals(expected.iTj, 1e-12))

    def check_g2o_round_trip(self, num_poses):
        """Check graph structure and Cartesian noise for one pose count."""
        dataset = LandmarkLocalizationDataset(num_poses)
        measurements = dataset.perturbed_measurements()
        expected_landmarks = {
            (landmark_measurement.k, L(landmark_measurement.l)): landmark_measurement
            for landmark_measurement in measurements.landmarks
        }
        expected_odometry = {
            (odometry_measurement.i, odometry_measurement.j): odometry_measurement
            for odometry_measurement in measurements.odometry
        }

        with TemporaryDirectory() as directory:
            filename = str(Path(directory) / "known_landmark_localization.g2o")
            dataset.write_g2o(filename, measurements)
            graph, values = gtsam.readG2o(filename, is3D=True)

        self.assertEqual(graph.size(), 5 * num_poses - 1)
        self.assertEqual(values.size(), num_poses + 4)
        for k, wTk in enumerate(dataset.wTks):
            self.assertTrue(values.atPose3(k).equals(wTk, 1e-5))
        for l, wL in enumerate(dataset.wLs):
            np.testing.assert_allclose(values.atPoint3(L(l)), wL)

        landmark_count = 0
        odometry_count = 0
        for index in range(graph.size()):
            factor = graph.at(index)
            keys = tuple(factor.keys())
            if isinstance(factor, gtsam.BearingRangeFactor3D):
                landmark_count += 1
                expected_landmark_measurement = expected_landmarks[keys]
                measured_kBearingRange = factor.measured()
                kRange = measured_kBearingRange.range()
                kBearing = measured_kBearingRange.bearing()
                DkP_dbearingRange = np.column_stack(
                    (kRange * kBearing.basis(), kBearing.unitVector())
                )
                measured_kP = kRange * kBearing.unitVector()
                kPCovariance = (
                    DkP_dbearingRange
                    @ factor.noiseModel().covariance()
                    @ DkP_dbearingRange.T
                )
                np.testing.assert_allclose(
                    measured_kP, expected_landmark_measurement.kP, rtol=1e-10
                )
                np.testing.assert_allclose(
                    kPCovariance,
                    np.linalg.inv(expected_landmark_measurement.information),
                    rtol=1e-9,
                    atol=1e-12,
                )
            elif isinstance(factor, gtsam.BetweenFactorPose3):
                odometry_count += 1
                expected_odometry_measurement = expected_odometry[keys]
                self.assertTrue(
                    factor.measured().equals(expected_odometry_measurement.iTj, 1e-5)
                )
            else:
                self.fail(f"Unexpected factor type: {type(factor).__name__}")

        self.assertEqual(landmark_count, 4 * num_poses)
        self.assertEqual(odometry_count, num_poses - 1)

    def test_g2o_round_trip(self):
        """Three- and 20-pose datasets round-trip through g2o."""
        for num_poses in (3, 20):
            with self.subTest(num_poses=num_poses):
                self.check_g2o_round_trip(num_poses)

    def test_committed_datasets_are_reproducible(self):
        """Committed g2o files match deterministic generator output."""
        datasets = (
            (3, "known_landmark_localization_3.g2o"),
            (20, "known_landmark_localization_20.g2o"),
        )
        with TemporaryDirectory() as directory:
            for num_poses, name in datasets:
                dataset = LandmarkLocalizationDataset(num_poses)
                generated = Path(directory) / name
                dataset.write_g2o(
                    str(generated), dataset.perturbed_measurements()
                )
                test_file = Path(__file__).resolve()
                package_data = test_file.parents[1] / "Data" / name
                committed = (
                    package_data
                    if package_data.is_file()
                    else test_file.parents[3] / "examples" / "Data" / name
                )
                self.assertEqual(generated.read_text(), committed.read_text())


if __name__ == "__main__":
    unittest.main()
