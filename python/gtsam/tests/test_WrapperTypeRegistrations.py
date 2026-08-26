"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Tests for C++ types exposed by generated wrapper signatures.
"""

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestWrapperTypeRegistrations(GtsamTestCase):
    """Checks that every public signature type has a usable Python class."""

    def test_camera_set_instantiations(self):
        """CameraSet instantiations accept each wrapped camera family."""
        camera_types = (
            ("CameraSetPinholePoseCal3DS2", "PinholePoseCal3DS2"),
            ("CameraSetPinholePoseCal3Unified", "PinholePoseCal3Unified"),
            ("CameraSetPinholePoseCal3Bundler", "PinholePoseCal3Bundler"),
            ("CameraSetPinholePoseCal3Fisheye", "PinholePoseCal3Fisheye"),
            ("CameraSetStereoCamera", "StereoCamera"),
        )

        for camera_set_name, camera_name in camera_types:
            with self.subTest(camera_set=camera_set_name):
                cameras = getattr(gtsam, camera_set_name)()
                camera = getattr(gtsam, camera_name)()
                cameras.push_back(camera)
                self.assertEqual(cameras.at(0).dim(), camera.dim())

    def test_pinhole_pose_cal3f_values_round_trip(self):
        """The Cal3f PinholePose specialization round-trips through Values."""
        camera = gtsam.PinholePoseCal3f(gtsam.Pose3())
        values = gtsam.Values()
        values.insert(0, camera)
        self.assertTrue(values.atPinholePoseCal3f(0).equals(camera, 1e-9))

    def test_empty_cal_spherical_camera_constructor(self):
        """The explicit EmptyCal constructor is callable from Python."""
        camera = gtsam.SphericalCamera(gtsam.Pose3(), gtsam.EmptyCal())
        self.gtsamAssertEquals(camera.pose(), gtsam.Pose3())

    def test_variable_slots_jacobian_constructor(self):
        """VariableSlots can be reused when assembling a JacobianFactor."""
        model = gtsam.noiseModel.Unit.Create(1)
        graph = gtsam.GaussianFactorGraph()
        graph.push_back(
            gtsam.JacobianFactor(0, np.ones((1, 1)), np.ones(1), model)
        )

        variable_slots = gtsam.VariableSlots(graph)
        combined = gtsam.JacobianFactor(graph, variable_slots)
        self.assertEqual(combined.rows(), 1)

    def test_vertical_block_matrix_constructor(self):
        """VerticalBlockMatrix can back the corresponding conditional overload."""
        augmented_matrix = gtsam.VerticalBlockMatrix(
            [1, 1], np.array([[2.0, 3.0]])
        )
        conditional = gtsam.GaussianConditional([0], 1, augmented_matrix)
        self.assertEqual(conditional.rows(), 1)

    def test_isam2_update_params(self):
        """ISAM2UpdateParams exposes scalar, container, and optional-map fields."""
        params = gtsam.ISAM2UpdateParams()
        self.assertFalse(params.force_relinearize)
        self.assertFalse(params.forceFullSolve)
        self.assertIsNone(params.newAffectedKeys)

        params.force_relinearize = True
        params.newAffectedKeys = {7: {1, 2}}
        self.assertEqual(params.newAffectedKeys, {7: {1, 2}})

        params.removeFactorIndices = [3]
        self.assertEqual(params.removeFactorIndices, [3])

        constrained_keys = gtsam.KeyGroupMap()
        constrained_keys.insert2(1, 2)
        params.constrainedKeys = constrained_keys
        self.assertEqual(params.constrainedKeys.at(1), 2)

        no_relin_keys = gtsam.KeyList()
        no_relin_keys.push_back(4)
        params.noRelinKeys = no_relin_keys
        self.assertEqual(params.noRelinKeys.front(), 4)

        extra_reelim_keys = gtsam.KeyList()
        extra_reelim_keys.push_back(5)
        params.extraReelimKeys = extra_reelim_keys
        self.assertEqual(params.extraReelimKeys.front(), 5)

        params.removeFactorIndices = []
        params.constrainedKeys = None
        params.noRelinKeys = None
        params.extraReelimKeys = None
        params.newAffectedKeys = None

        result = gtsam.ISAM2().update(
            gtsam.NonlinearFactorGraph(), gtsam.Values(), params
        )
        self.assertIsInstance(result, gtsam.ISAM2Result)

    def test_hybrid_gaussian_conditional_conditionals(self):
        """Conditionals can construct a multi-key hybrid conditional."""
        first_mode_key = 10
        second_mode_key = 11
        continuous_key = 20
        conditionals = [
            gtsam.GaussianConditional.FromMeanAndStddev(
                continuous_key, np.array([mean]), 1.0
            )
            for mean in range(4)
        ]
        conditional_tree = gtsam.HybridGaussianConditionalConditionals(
            [(first_mode_key, 2), (second_mode_key, 2)], conditionals
        )

        self.assertFalse(conditional_tree.empty())
        self.assertEqual(conditional_tree.nrLeaves(), 4)

        assignment = gtsam.DiscreteValues()
        assignment[first_mode_key] = 0
        assignment[second_mode_key] = 0
        np.testing.assert_allclose(conditional_tree(assignment).d(), [0.0])

        discrete_keys = gtsam.DiscreteKeys()
        discrete_keys.push_back((first_mode_key, 2))
        discrete_keys.push_back((second_mode_key, 2))
        conditional = gtsam.HybridGaussianConditional(
            discrete_keys, conditional_tree
        )
        self.assertEqual(conditional.nrComponents(), 4)
        np.testing.assert_allclose(conditional.choose(assignment).d(), [0.0])

        assignment[first_mode_key] = 1
        assignment[second_mode_key] = 1
        np.testing.assert_allclose(conditional_tree(assignment).d(), [3.0])
        np.testing.assert_allclose(conditional.choose(assignment).d(), [3.0])

    def test_hybrid_nonlinear_factor_value_pairs(self):
        """FactorValuePairs can construct a multi-key hybrid factor."""
        first_mode_key = 10
        second_mode_key = 11
        continuous_key = 20
        prior = gtsam.PriorFactorPose2(
            continuous_key,
            gtsam.Pose2(),
            gtsam.noiseModel.Unit.Create(3),
        )
        factor_values = [
            (prior, 1.25),
            (prior, 2.5),
            (prior, 3.75),
            (prior, 5.0),
        ]
        factor_tree = gtsam.HybridNonlinearFactorValuePairs(
            [(first_mode_key, 2), (second_mode_key, 2)], factor_values
        )

        self.assertFalse(factor_tree.empty())
        self.assertEqual(factor_tree.nrLeaves(), 4)

        assignment = gtsam.DiscreteValues()
        assignment[first_mode_key] = 0
        assignment[second_mode_key] = 0
        self.assertEqual(factor_tree(assignment)[1], 1.25)

        discrete_keys = gtsam.DiscreteKeys()
        discrete_keys.push_back((first_mode_key, 2))
        discrete_keys.push_back((second_mode_key, 2))
        factor = gtsam.HybridNonlinearFactor(discrete_keys, factor_tree)

        values = gtsam.Values()
        values.insert(continuous_key, gtsam.Pose2())
        self.assertEqual(factor.error(values, assignment), 1.25)

        assignment[first_mode_key] = 1
        assignment[second_mode_key] = 1
        self.assertEqual(factor_tree(assignment)[1], 5.0)
        self.assertEqual(factor.error(values, assignment), 5.0)


if __name__ == "__main__":
    unittest.main()
