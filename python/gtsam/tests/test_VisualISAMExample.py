"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

visual_isam unit tests.
Author: Frank Dellaert & Duy Nguyen Ta & Varun Agrawal (Python)
"""
# pylint: disable=maybe-no-member,invalid-name

import unittest

import gtsam.utils.visual_data_generator as generator
import gtsam.utils.visual_isam as visual_isam
from gtsam.utils.test_case import GtsamTestCase

from gtsam import symbol


class TestVisualISAMExample(GtsamTestCase):
    """Test class for ISAM2 with visual landmarks."""
    def test_VisualISAMExample(self):
        """Test to see if ISAM works as expected for a simple visual SLAM example."""
        # Data Options
        options = generator.Options()
        options.triangle = False
        options.nrCameras = 20

        # iSAM Options
        isamOptions = visual_isam.Options()
        isamOptions.hardConstraint = False
        isamOptions.pointPriors = False
        isamOptions.batchInitialization = True
        isamOptions.reorderInterval = 10
        isamOptions.alwaysRelinearize = False

        # Generate data
        data, truth = generator.generate_data(options)

        # Initialize iSAM with the first pose and points
        isam, result, nextPose = visual_isam.initialize(
            data, truth, isamOptions)

        # Main loop for iSAM: stepping through all poses
        for currentPose in range(nextPose, options.nrCameras):
            isam, result = visual_isam.step(data, isam, result, truth,
                                            currentPose)

        for i, _ in enumerate(truth.cameras):
            pose_i = result.atPose3(symbol('x', i))
            self.gtsamAssertEquals(pose_i, truth.cameras[i].pose(), 1e-5)

        for j, _ in enumerate(truth.points):
            point_j = result.atPoint3(symbol('l', j))
            self.gtsamAssertEquals(point_j, truth.points[j], 1e-5)

    def test_isam2_update(self):
        """
        Test for full version of ISAM2::update method
        """
        # Data Options
        options = generator.Options()
        options.triangle = False
        options.nrCameras = 20

        # iSAM Options
        isamOptions = visual_isam.Options()
        isamOptions.hardConstraint = False
        isamOptions.pointPriors = False
        isamOptions.batchInitialization = True
        isamOptions.reorderInterval = 10
        isamOptions.alwaysRelinearize = False

        # Generate data
        data, truth = generator.generate_data(options)

        # Initialize iSAM with the first pose and points
        isam, result, nextPose = visual_isam.initialize(data, truth, isamOptions)

        remove_factor_indices = gtsam.FactorIndices()
        constrained_keys = gtsam.KeyGroupMap()
        no_relin_keys = gtsam.KeyList()
        extra_reelim_keys = gtsam.KeyList()
        isamArgs = (remove_factor_indices,
                    constrained_keys,
                    no_relin_keys,
                    extra_reelim_keys,
                    False)

        # Main loop for iSAM: stepping through all poses
        for currentPose in range(nextPose, options.nrCameras):
            isam, result = visual_isam.step(data, isam, result, truth, currentPose, isamArgs)

        for i in range(len(truth.cameras)):
            pose_i = result.atPose3(symbol(ord('x'), i))
            self.gtsamAssertEquals(pose_i, truth.cameras[i].pose(), 1e-5)

        for j in range(len(truth.points)):
            point_j = result.atPoint3(symbol(ord('l'), j))
            self.gtsamAssertEquals(point_j, truth.points[j], 1e-5)


if __name__ == "__main__":
    unittest.main()
