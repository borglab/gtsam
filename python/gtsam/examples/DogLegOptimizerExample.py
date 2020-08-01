"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Example comparing DoglegOptimizer with Levenberg-Marquardt.
Author: Frank Dellaert
"""
# pylint: disable=no-member, invalid-name

import math
import argparse

import gtsam
import matplotlib.pyplot as plt
import numpy as np


def run(args):
    """Test Dogleg vs LM, inspired by issue #452."""

    # print parameters
    print("num samples = {}, deltaInitial = {}".format(
        args.num_samples, args.delta))

    # Ground truth solution
    T11 = gtsam.Pose2(0, 0, 0)
    T12 = gtsam.Pose2(1, 0, 0)
    T21 = gtsam.Pose2(0, 1, 0)
    T22 = gtsam.Pose2(1, 1, 0)

    # Factor graph
    graph = gtsam.NonlinearFactorGraph()

    # Priors
    prior = gtsam.noiseModel.Isotropic.Sigma(3, 1)
    graph.add(gtsam.PriorFactorPose2(11, T11, prior))
    graph.add(gtsam.PriorFactorPose2(21, T21, prior))

    # Odometry
    model = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.3]))
    graph.add(gtsam.BetweenFactorPose2(11, 12, T11.between(T12), model))
    graph.add(gtsam.BetweenFactorPose2(21, 22, T21.between(T22), model))

    # Range
    model_rho = gtsam.noiseModel.Isotropic.Sigma(1, 0.01)
    graph.add(gtsam.RangeFactorPose2(12, 22, 1.0, model_rho))

    params = gtsam.DoglegParams()
    params.setDeltaInitial(args.delta)  # default is 10

    # Add progressively more noise to ground truth
    sigmas = [0.01, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20]
    n = len(sigmas)
    p_dl, s_dl, p_lm, s_lm = [0]*n, [0]*n, [0]*n, [0]*n
    for i, sigma in enumerate(sigmas):
        dl_fails, lm_fails = 0, 0
        # Attempt num_samples optimizations for both DL and LM
        for _attempt in range(args.num_samples):
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
        v = args.num_samples+alpha+beta
        p_dl[i] = (args.num_samples-dl_fails+alpha)/v
        p_lm[i] = (args.num_samples-lm_fails+alpha)/v

        def stddev(p):
            """Calculate standard deviation."""
            return math.sqrt(p*(1-p)/(1+v))

        s_dl[i] = stddev(p_dl[i])
        s_lm[i] = stddev(p_lm[i])

        fmt = "sigma= {}:\tDL success {:.2f}% +/- {:.2f}%, LM success {:.2f}% +/- {:.2f}%"
        print(fmt.format(sigma,
                         100*p_dl[i], 100*s_dl[i],
                         100*p_lm[i], 100*s_lm[i]))

    if args.plot:
        fig, ax = plt.subplots()
        dl_plot = plt.errorbar(sigmas, p_dl, yerr=s_dl, label="Dogleg")
        lm_plot = plt.errorbar(sigmas, p_lm, yerr=s_lm, label="LM")
        plt.title("Dogleg emprical success vs. LM")
        plt.legend(handles=[dl_plot, lm_plot])
        ax.set_xlim(0, sigmas[-1]+1)
        ax.set_ylim(0, 1)
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compare Dogleg and LM success rates")
    parser.add_argument("-n", "--num_samples", type=int, default=1000,
                        help="Number of samples for each sigma")
    parser.add_argument("-d", "--delta", type=float, default=10.0,
                        help="Initial delta for dogleg")
    parser.add_argument("-p", "--plot", action="store_true",
                        help="Flag to plot results")
    args = parser.parse_args()
    run(args)
