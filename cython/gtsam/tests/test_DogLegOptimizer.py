"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

DoglegOptimizer unit tests.
Author: Frank Dellaert
"""
# pylint: disable=no-member, invalid-name

import math
import unittest

import gtsam
import matplotlib.pyplot as plt
import numpy as np
from gtsam.utils.test_case import GtsamTestCase


class TestDoglegOptimizer(GtsamTestCase):
    """Test Dogleg vs LM, isnpired by issue #452."""

    def test_DoglegOptimizer(self):
        # Ground truth solution
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
        model = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.01, 0.01, 0.3]))
        graph.add(gtsam.BetweenFactorPose2(11, 12, T11.between(T12), model))
        graph.add(gtsam.BetweenFactorPose2(21, 22, T21.between(T22), model))

        # Range
        model_rho = gtsam.noiseModel_Isotropic.Sigma(1, 0.01)
        graph.add(gtsam.RangeFactorPose2(12, 22, 1.0, model_rho))

        num_samples = 1000
        print("num samples = {}%".format(num_samples))

        params = gtsam.DoglegParams()
        params.setDeltaInitial(10)  # default was 1.0

        # Add progressively more noise to ground truth
        sigmas = [0.01, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20]
        n = len(sigmas)
        p_dl, s_dl, p_lm, s_lm = [0]*n, [0]*n, [0]*n, [0]*n
        for i, sigma in enumerate(sigmas):
            dl_fails, lm_fails = 0, 0
            # Attempt num_samples optimizations for both DL and LM
            for _attempt in range(num_samples):
                initial = gtsam.Values()
                initial.insert(11, T11.retract(np.random.normal(0, sigma, 3)))
                initial.insert(12, T12.retract(np.random.normal(0, sigma, 3)))
                initial.insert(21, T21.retract(np.random.normal(0, sigma, 3)))
                initial.insert(22, T22.retract(np.random.normal(0, sigma, 3)))

                # Run dogleg optimizer
                dl = gtsam.DoglegOptimizer(graph, initial, params)
                result = dl.optimize()
                dl_fails += graph.error(result) > 1e-9

                # Run
                lm = gtsam.LevenbergMarquardtOptimizer(graph, initial)
                result = lm.optimize()
                lm_fails += graph.error(result) > 1e-9

            # Calculate Bayes estimate of success probability
            # using a beta prior of alpha=0.5, beta=0.5
            alpha, beta = 0.5, 0.5
            v = num_samples+alpha+beta
            p_dl[i] = (num_samples-dl_fails+alpha)/v
            p_lm[i] = (num_samples-lm_fails+alpha)/v

            def stddev(p):
                """Calculate standard deviation."""
                return math.sqrt(p*(1-p)/(1+v))

            s_dl[i] = stddev(p_dl[i])
            s_lm[i] = stddev(p_lm[i])

            fmt = "sigma= {}:\tDL success {:.2f}% +/- {:.2f}%, LM success {:.2f}% +/- {:.2f}%"
            print(fmt.format(sigma,
                             100*p_dl[i], 100*s_dl[i],
                             100*p_lm[i], 100*s_lm[i]))

        fig, ax = plt.subplots()
        dl_plot = plt.errorbar(sigmas, p_dl, yerr=s_dl, label="Dogleg")
        lm_plot = plt.errorbar(sigmas, p_lm, yerr=s_lm, label="LM")
        plt.title("Dogleg emprical success vs. LM")
        plt.legend(handles=[dl_plot, lm_plot])
        ax.set_xlim(0, sigmas[-1]+1)
        ax.set_ylim(0, 1)
        plt.show()


if __name__ == "__main__":
    unittest.main()
