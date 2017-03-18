import unittest
import gtsam
from gtsam import symbol
import numpy as np
import gtsam.utils.visual_data_generator as generator


class TestSFMExample(unittest.TestCase):

    def test_SFMExample(self):
        options = generator.Options()
        options.triangle = False
        options.nrCameras = 10

        [data, truth] = generator.generate_data(options)

        measurementNoiseSigma = 1.0
        pointNoiseSigma = 0.1
        poseNoiseSigmas = np.array([0.001, 0.001, 0.001, 0.1, 0.1, 0.1])

        graph = gtsam.NonlinearFactorGraph()

        # Add factors for all measurements
        measurementNoise = gtsam.noiseModel_Isotropic.Sigma(2, measurementNoiseSigma)
        for i in range(len(data.Z)):
            for k in range(len(data.Z[i])):
                j = data.J[i][k]
                graph.add(gtsam.GenericProjectionFactorCal3_S2(
                    data.Z[i][k], measurementNoise,
                    symbol(ord('x'), i), symbol(ord('p'), j), data.K))

        posePriorNoise = gtsam.noiseModel_Diagonal.Sigmas(poseNoiseSigmas)
        graph.add(gtsam.PriorFactorPose3(symbol(ord('x'), 0),
                                   truth.cameras[0].pose(), posePriorNoise))
        pointPriorNoise = gtsam.noiseModel_Isotropic.Sigma(3, pointNoiseSigma)
        graph.add(gtsam.PriorFactorPoint3(symbol(ord('p'), 0),
                                    truth.points[0], pointPriorNoise))

        # Initial estimate
        initialEstimate = gtsam.Values()
        for i in range(len(truth.cameras)):
            pose_i = truth.cameras[i].pose()
            initialEstimate.insert(symbol(ord('x'), i), pose_i)
        for j in range(len(truth.points)):
            point_j = truth.points[j]
            initialEstimate.insert(symbol(ord('p'), j), point_j)

        # Optimization
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate)
        for i in range(5):
            optimizer.iterate()
        result = optimizer.values()

        # Marginalization
        marginals = gtsam.Marginals(graph, result)
        marginals.marginalCovariance(symbol(ord('p'), 0))
        marginals.marginalCovariance(symbol(ord('x'), 0))

        # Check optimized results, should be equal to ground truth
        for i in range(len(truth.cameras)):
            pose_i = result.atPose3(symbol(ord('x'), i))
            self.assertTrue(pose_i.equals(truth.cameras[i].pose(), 1e-5))

        for j in range(len(truth.points)):
            point_j = result.atPoint3(symbol(ord('p'), j))
            self.assertTrue(point_j.equals(truth.points[j], 1e-5))

if __name__ == "__main__":
    unittest.main()
