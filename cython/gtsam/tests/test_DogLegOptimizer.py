"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

DoglegOptimizer unit tests.
Author: Frank Dellaert
"""
# pylint: disable=no-member, invalid-name
import unittest

import gtsam
import numpy as np
from gtsam.utils.test_case import GtsamTestCase


class TestDoglegOptimizer(GtsamTestCase):

    def test_DoglegOptimizer(self):
        # Linearization point
        T11 = gtsam.Pose2(0, 0, 0)
        T12 = gtsam.Pose2(1, 0, 0)
        T21 = gtsam.Pose2(0, 1, 0)
        T22 = gtsam.Pose2(1, 1, 0)

        # Factor graph
        graph = gtsam.NonlinearFactorGraph()

        # Priors
        prior = gtsam.noiseModel_Isotropic.Sigma(3, 1)
        graph.add(gtsam.PriorFactorPose2(11, T11, prior))
        graph.add(gtsam.PriorFactorPose2(21, T21, prior))

        # Odometry
        model = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.01, 0.01, 1e6]))
        graph.add(gtsam.BetweenFactorPose2(11, 12, T11.between(T12), model))
        graph.add(gtsam.BetweenFactorPose2(21, 22, T21.between(T22), model))

        # Range
        model_rho = gtsam.noiseModel_Isotropic.Sigma(1, 0.01)
        graph.add(gtsam.RangeFactorPose2(12, 22, 1.0, model_rho))

        # Print graph
        print(graph)

        sigma = 0.1
        values = gtsam.Values()
        values.insert(11, T11.retract(np.random.normal(0, sigma, 3)))
        values.insert(12, T12)
        values.insert(21, T21)
        values.insert(22, T22)
        linearized = graph.linearize(values)

        # Get Jacobian
        ordering = gtsam.Ordering()
        ordering.push_back(11)
        ordering.push_back(21)
        ordering.push_back(12)
        ordering.push_back(22)
        A, b = linearized.jacobian(ordering)
        Q = np.dot(A.transpose(), A)
        print(np.linalg.det(Q))

        bn = linearized.eliminateSequential(ordering)

        # Print gradient
        linearized.gradientAtZero()

        # Run dogleg optimizer
        dl = gtsam.DoglegOptimizer(graph, values)
        result = dl.optimize()
        print(graph.error(result))


if __name__ == "__main__":
    unittest.main()
