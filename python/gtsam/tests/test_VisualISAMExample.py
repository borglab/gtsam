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

    def setUp(self):
        # Data Options
        options = generator.Options()
        options.triangle = False
        options.nrCameras = 20
        self.options = options

        # iSAM Options
        isamOptions = visual_isam.Options()
        isamOptions.hardConstraint = False
        isamOptions.pointPriors = False
        isamOptions.batchInitialization = True
        isamOptions.reorderInterval = 10
        isamOptions.alwaysRelinearize = False
        self.isamOptions = isamOptions

        # Generate data
        self.data, self.truth = generator.generate_data(options)

    def test_VisualISAMExample(self):
        """Test to see if ISAM works as expected for a simple visual SLAM example."""
        # Initialize iSAM with the first pose and points
        isam, result, nextPose = visual_isam.initialize(
            self.data, self.truth, self.isamOptions)

        # Main loop for iSAM: stepping through all poses
        for currentPose in range(nextPose, self.options.nrCameras):
            isam, result = visual_isam.step(self.data, isam, result,
                                            self.truth, currentPose)

        for i, true_camera in enumerate(self.truth.cameras):
            pose_i = result.atPose3(symbol('x', i))
            self.gtsamAssertEquals(pose_i, true_camera.pose(), 1e-5)

        for j, expected_point in enumerate(self.truth.points):
            point_j = result.atPoint3(symbol('l', j))
            self.gtsamAssertEquals(point_j, expected_point, 1e-5)

    @unittest.skip(
        "Need to understand how to calculate error using VectorValues correctly"
    )
    def test_isam2_error(self):
        #TODO(Varun) fix
        # Initialize iSAM with the first pose and points
        isam, result, nextPose = visual_isam.initialize(
            self.data, self.truth, self.isamOptions)

        # Main loop for iSAM: stepping through all poses
        for currentPose in range(nextPose, self.options.nrCameras):
            isam, result = visual_isam.step(self.data, isam, result,
                                            self.truth, currentPose)

        values = gtsam.VectorValues()

        estimate = isam.calculateBestEstimate()

        keys = estimate.keys()

        for k in range(keys.size()):
            key = keys.at(k)
            try:
                v = estimate.atPose3(key).matrix()

            except RuntimeError:
                v = estimate.atPoint3(key).vector()
            values.insert(key, v)
        # print(isam.error(values))

    def test_isam2_update(self):
        """
        Test for full version of ISAM2::update method
        """
        # Initialize iSAM with the first pose and points
        isam, result, nextPose = visual_isam.initialize(
            self.data, self.truth, self.isamOptions)

        remove_factor_indices = gtsam.FactorIndices()
        constrained_keys = gtsam.KeyGroupMap()
        no_relin_keys = gtsam.KeyList()
        extra_reelim_keys = gtsam.KeyList()
        isamArgs = (remove_factor_indices, constrained_keys, no_relin_keys,
                    extra_reelim_keys, False)

        # Main loop for iSAM: stepping through all poses
        for currentPose in range(nextPose, self.options.nrCameras):
            isam, result = visual_isam.step(self.data, isam, result,
                                            self.truth, currentPose, isamArgs)

        for i in range(len(self.truth.cameras)):
            pose_i = result.atPose3(symbol(ord('x'), i))
            self.gtsamAssertEquals(pose_i, self.truth.cameras[i].pose(), 1e-5)

        for j in range(len(self.truth.points)):
            point_j = result.atPoint3(symbol(ord('l'), j))
            self.gtsamAssertEquals(point_j, self.truth.points[j], 1e-5)


if __name__ == "__main__":
    unittest.main()
