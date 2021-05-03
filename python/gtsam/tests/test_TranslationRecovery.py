from __future__ import print_function

import numpy as np
import unittest

import gtsam

""" Returns example pose values of 3 points A, B and C in the world frame """
def ExampleValues():
    T = []
    T.append(gtsam.Point3(np.array([3.14, 1.59, 2.65])))
    T.append(gtsam.Point3(np.array([-1.0590e+00, -3.6017e-02, -1.5720e+00])))
    T.append(gtsam.Point3(np.array([8.5034e+00, 6.7499e+00, -3.6383e+00])))
    
    data = gtsam.Values()
    for i in range(len(T)):
        data.insert(i, gtsam.Pose3(gtsam.Rot3(), T[i]))
    return data

""" Returns binary measurements for the points in the given edges."""
def SimulateMeasurements(gt_poses, graph_edges):
    measurements = gtsam.BinaryMeasurementsUnit3()
    for edge in graph_edges:
        Ta = gt_poses.atPose3(edge[0]).translation()
        Tb = gt_poses.atPose3(edge[1]).translation()
        measurements.append(gtsam.BinaryMeasurementUnit3( \
            edge[0], edge[1], gtsam.Unit3(Tb - Ta), \
            gtsam.noiseModel.Isotropic.Sigma(3, 0.01)))
    return measurements

""" Tests for the translation recovery class """
class TestTranslationRecovery(unittest.TestCase):
    """Test selected Translation Recovery methods."""

    def test_constructor(self):
        """Construct from binary measurements."""
        algorithm = gtsam.TranslationRecovery(gtsam.BinaryMeasurementsUnit3())
        self.assertIsInstance(algorithm, gtsam.TranslationRecovery)

    def test_run(self):
        gt_poses = ExampleValues()
        measurements = SimulateMeasurements(gt_poses, [[0, 1], [0, 2], [1, 2]])

        # Set verbosity to Silent for tests
        lmParams = gtsam.LevenbergMarquardtParams()
        lmParams.setVerbosityLM("silent")

        algorithm = gtsam.TranslationRecovery(measurements, lmParams)
        scale = 2.0
        result = algorithm.run(scale)

        w_aTc = gt_poses.atPose3(2).translation() - gt_poses.atPose3(0).translation()
        w_aTb = gt_poses.atPose3(1).translation() - gt_poses.atPose3(0).translation()
        w_aTc_expected = w_aTc*scale/np.linalg.norm(w_aTb)
        w_aTb_expected = w_aTb*scale/np.linalg.norm(w_aTb)

        np.testing.assert_array_almost_equal(result.atPoint3(0), np.array([0,0,0]), 6, "Origin result is incorrect.")
        np.testing.assert_array_almost_equal(result.atPoint3(1), w_aTb_expected, 6, "Point B result is incorrect.")
        np.testing.assert_array_almost_equal(result.atPoint3(2), w_aTc_expected, 6, "Point C result is incorrect.")

if __name__ == "__main__":
    unittest.main()

