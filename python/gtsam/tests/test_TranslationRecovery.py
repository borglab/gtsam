from __future__ import print_function

import numpy as np
import unittest

import gtsam

def SimulateMeasurements(gt_poses, graph_edges):
    measurements = gtsam.BinaryMeasurementsUnit3()
    for edge in graph_edges:
        Ta = gt_poses.atPose3(edge[0]).translation()
        Tb = gt_poses.atPose3(edge[1]).translation()
        measurements.append(BinaryMeasurementUnit3( \
            edge[0], edge[1], gtsam.Unit3(Tb - Ta), \
            gtsam.noiseModel.Isotropic.Sigma(3, 0.01)))
    return measurements

# Hard-coded values from dubrovnik-3-7-pre.txt
def ExampleValues():
    T = []
    T.append(gtsam.Point3(np.array([7.3030e-01, -2.6490e-01, -1.7127e+00])))
    T.append(gtsam.Point3(np.array([-1.0590e+00, -3.6017e-02, -1.5720e+00])))
    T.append(gtsam.Point3(np.array([8.5034e+00, 6.7499e+00, -3.6383e+00])))
    
    data = gtsam.Values()
    for i in range(len(T)):
        data.insert(i, gtsam.Pose3(R[i], T[i]))
    return data

class TestTranslationRecovery(unittest.TestCase):
    """Test selected Translation Recovery methods."""

    def test_constructor(self):
        """Construct from binary measurements."""
        algorithm = gtsam.TranslationRecovery(gtsam.BinaryMeasurementsUnit3())
        self.assertIsInstance(algorithm, gtsam.TranslationRecovery)

    def test_run(self):
        gt_poses = ExampleValues()
        measurements = SimulateMeasurements(gt_poses, [[0, 1], [0, 2], [1, 2]])
        algorithm = gtsam.TranslationRecovery(measurements)
        result = algorithm.run(2.0)
        for i in range(3):
            self.gtsamAssertEquals(result.atPoint3(i), 2*gt_poses.atPose3(i).translation(), 1e-6)

if __name__ == "__main__":
    unittest.main()

