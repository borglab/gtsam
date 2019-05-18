"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

A script validating the ImuFactor inference.
"""

from __future__ import print_function

import math

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import gtsam
from gtsam.utils.plot import plot_pose3
from PreintegrationExample import POSES_FIG, PreintegrationExample

BIAS_KEY = int(gtsam.symbol(ord('b'), 0))


def X(key):
    """Create symbol for pose key."""
    return gtsam.symbol(ord('x'), key)


def V(key):
    """Create symbol for velocity key."""
    return gtsam.symbol(ord('v'), key)


np.set_printoptions(precision=3, suppress=True)


class ImuFactorExample(PreintegrationExample):

    def __init__(self):
        self.velocity = np.array([2, 0, 0])
        self.priorNoise = gtsam.noiseModel_Isotropic.Sigma(6, 0.1)
        self.velNoise = gtsam.noiseModel_Isotropic.Sigma(3, 0.1)

        # Choose one of these twists to change scenario:
        zero_twist = (np.zeros(3), np.zeros(3))
        forward_twist = (np.zeros(3), self.velocity)
        loop_twist = (np.array([0, -math.radians(30), 0]), self.velocity)
        sick_twist = (
            np.array([math.radians(30), -math.radians(30), 0]), self.velocity)

        accBias = np.array([-0.3, 0.1, 0.2])
        gyroBias = np.array([0.1, 0.3, -0.1])
        bias = gtsam.imuBias_ConstantBias(accBias, gyroBias)

        dt = 1e-2
        super(ImuFactorExample, self).__init__(sick_twist, bias, dt)

    def addPrior(self, i, graph):
        state = self.scenario.navState(i)
        graph.push_back(gtsam.PriorFactorPose3(
            X(i), state.pose(), self.priorNoise))
        graph.push_back(gtsam.PriorFactorVector(
            V(i), state.velocity(), self.velNoise))

    def run(self):
        graph = gtsam.NonlinearFactorGraph()

        # initialize data structure for pre-integrated IMU measurements
        pim = gtsam.PreintegratedImuMeasurements(self.params, self.actualBias)

        T = 12
        num_poses = T + 1  # assumes 1 factor per second
        initial = gtsam.Values()
        initial.insert(BIAS_KEY, self.actualBias)
        for i in range(num_poses):
            state_i = self.scenario.navState(float(i))
            initial.insert(X(i), state_i.pose())
            initial.insert(V(i), state_i.velocity())

        # simulate the loop
        i = 0  # state index
        actual_state_i = self.scenario.navState(0)
        for k, t in enumerate(np.arange(0, T, self.dt)):
            # get measurements and add them to PIM
            measuredOmega = self.runner.measuredAngularVelocity(t)
            measuredAcc = self.runner.measuredSpecificForce(t)
            pim.integrateMeasurement(measuredAcc, measuredOmega, self.dt)

            # Plot IMU many times
            if k % 10 == 0:
                self.plotImu(t, measuredOmega, measuredAcc)

            # Plot every second
            if k % int(1 / self.dt) == 0:
                self.plotGroundTruthPose(t)

            # create IMU factor every second
            if (k + 1) % int(1 / self.dt) == 0:
                factor = gtsam.ImuFactor(X(i), V(i), X(
                    i + 1), V(i + 1), BIAS_KEY, pim)
                graph.push_back(factor)
                if True:
                    print(factor)
                    print(pim.predict(actual_state_i, self.actualBias))
                pim.resetIntegration()
                actual_state_i = self.scenario.navState(t + self.dt)
                i += 1

        # add priors on beginning and end
        self.addPrior(0, graph)
        self.addPrior(num_poses - 1, graph)

        # optimize using Levenberg-Marquardt optimization
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM("SUMMARY")
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
        result = optimizer.optimize()

        # Calculate and print marginal covariances
        marginals = gtsam.Marginals(graph, result)
        print("Covariance on bias:\n", marginals.marginalCovariance(BIAS_KEY))
        for i in range(num_poses):
            print("Covariance on pose {}:\n{}\n".format(
                i, marginals.marginalCovariance(X(i))))
            print("Covariance on vel {}:\n{}\n".format(
                i, marginals.marginalCovariance(V(i))))

        # Plot resulting poses
        i = 0
        while result.exists(X(i)):
            pose_i = result.atPose3(X(i))
            plot_pose3(POSES_FIG, pose_i, 0.1)
            i += 1
        print(result.atimuBias_ConstantBias(BIAS_KEY))

        plt.ioff()
        plt.show()


if __name__ == '__main__':
    ImuFactorExample().run()
