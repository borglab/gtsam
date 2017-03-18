import unittest
import numpy as np
from gtsam import symbol
import gtsam.utils.visual_data_generator as generator
import gtsam.utils.visual_isam as visual_isam

class TestVisualISAMExample(unittest.TestCase):

    def test_VisualISAMExample(self):
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

        # Main loop for iSAM: stepping through all poses
        for currentPose in range(nextPose, options.nrCameras):
            isam, result = visual_isam.step(data, isam, result, truth, currentPose)

        for i in range(len(truth.cameras)):
            pose_i = result.atPose3(symbol(ord('x'), i))
            self.assertTrue(pose_i.equals(truth.cameras[i].pose(), 1e-5))

        for j in range(len(truth.points)):
            point_j = result.atPoint3(symbol(ord('l'), j))
            self.assertTrue(point_j.equals(truth.points[j], 1e-5))

if __name__ == "__main__":
    unittest.main()
