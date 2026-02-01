"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for TrajectoryAlignerSim3.
Author: Akshay Krishnan
"""
import unittest

import gtsam
from gtsam.utils.test_case import GtsamTestCase


def make_parent_poses():
    return [
        gtsam.Pose3(),
        gtsam.Pose3(
            gtsam.Rot3.RzRyRx(0.15, -0.2, 0.1), gtsam.Point3(1.0, 0.1, 0.0)
        ),
        gtsam.Pose3(
            gtsam.Rot3.RzRyRx(0.1, 0.05, -0.03), gtsam.Point3(2.0, 0.4, 0.1)
        ),
    ]


def make_measurements(poses, sigma=1e-3):
    noise = gtsam.noiseModel.Isotropic.Sigma(6, sigma)
    return [
        gtsam.UnaryMeasurementPose3(i, pose, noise)
        for i, pose in enumerate(poses)
    ]


def transform_poses(sim, poses):
    return [sim.transformFrom(p) for p in poses]


class TestTrajectoryAlignerSim3(GtsamTestCase):
    """Tests for TrajectoryAlignerSim3."""

    def test_perfect_alignment_without_initial_sim(self):
        parent = make_parent_poses()
        gt_bSa = gtsam.Similarity3(
            gtsam.Rot3.RzRyRx(0.2, -0.1, 0.05),
            gtsam.Point3(0.3, -0.2, 0.1),
            1.4,
        )
        child = transform_poses(gt_bSa, parent)

        aTi = make_measurements(parent)
        bTi_all = [make_measurements(child)]

        aligner = gtsam.TrajectoryAlignerSim3(aTi, bTi_all)
        result = aligner.solve()

        recovered = result.atSimilarity3(gtsam.Symbol("S", 0).key())
        self.assertTrue(gt_bSa.equals(recovered, 1e-6))

    def test_noisy_alignment_with_initial_guess(self):
        parent = make_parent_poses()
        gt_bSa = gtsam.Similarity3(
            gtsam.Rot3.RzRyRx(-0.15, 0.05, 0.08),
            gtsam.Point3(-0.2, 0.15, 0.25),
            1.3,
        )
        child = transform_poses(gt_bSa, parent)

        aTi = make_measurements(parent, sigma=1e-2)
        bTi_all = [make_measurements(child, sigma=1e-2)]

        init_guess = gtsam.Similarity3(
            gtsam.Rot3.RzRyRx(0.05, -0.04, 0.02),
            gtsam.Point3(0.5, -0.3, 0.1),
            0.8,
        )
        aligner = gtsam.TrajectoryAlignerSim3(aTi, bTi_all, [init_guess])
        result = aligner.solve()

        recovered = result.atSimilarity3(gtsam.Symbol("S", 0).key())
        self.assertTrue(gt_bSa.equals(recovered, 1e-2))

        marginals = aligner.marginalize(result)
        self.assertTrue(marginals.marginalInformation(gtsam.Symbol("S", 0).key()).size > 0)


if __name__ == "__main__":
    unittest.main()
