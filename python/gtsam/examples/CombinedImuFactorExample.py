"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

A script validating and demonstrating inference with the CombinedImuFactor.

Author: Varun Agrawal
"""

# pylint: disable=no-name-in-module,unused-import,arguments-differ,import-error,wrong-import-order

from __future__ import print_function

import argparse
import math

import matplotlib.pyplot as plt
import numpy as np
from gtsam.symbol_shorthand import B, V, X
from gtsam.utils.plot import plot_pose3
from mpl_toolkits.mplot3d import Axes3D
from PreintegrationExample import POSES_FIG, PreintegrationExample

import gtsam

GRAVITY = 9.81

np.set_printoptions(precision=3, suppress=True)


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser("CombinedImuFactorExample.py")
    parser.add_argument("--twist_scenario",
                        default="sick_twist",
                        choices=("zero_twist", "forward_twist", "loop_twist",
                                 "sick_twist"))
    parser.add_argument("--time",
                        "-T",
                        default=12,
                        type=int,
                        help="Total navigation time in seconds")
    parser.add_argument("--compute_covariances",
                        default=False,
                        action='store_true')
    parser.add_argument("--verbose", default=False, action='store_true')
    return parser.parse_args()


class CombinedImuFactorExample(PreintegrationExample):
    """Class to run example of the Imu Factor."""
    def __init__(self, twist_scenario: str = "sick_twist"):
        self.velocity = np.array([2, 0, 0])
        self.priorNoise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        self.velNoise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        self.biasNoise = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)

        # Choose one of these twists to change scenario:
        twist_scenarios = dict(
            zero_twist=(np.zeros(3), np.zeros(3)),
            forward_twist=(np.zeros(3), self.velocity),
            loop_twist=(np.array([0, -math.radians(30), 0]), self.velocity),
            sick_twist=(np.array([math.radians(30), -math.radians(30),
                                  0]), self.velocity))

        accBias = np.array([-0.3, 0.1, 0.2])
        gyroBias = np.array([0.1, 0.3, -0.1])
        bias = gtsam.imuBias.ConstantBias(accBias, gyroBias)

        params = gtsam.PreintegrationCombinedParams.MakeSharedU(GRAVITY)

        # Some arbitrary noise sigmas
        gyro_sigma = 1e-3
        accel_sigma = 1e-3
        I_3x3 = np.eye(3)
        params.setGyroscopeCovariance(gyro_sigma**2 * I_3x3)
        params.setAccelerometerCovariance(accel_sigma**2 * I_3x3)
        params.setIntegrationCovariance(1e-7**2 * I_3x3)

        dt = 1e-2
        super(CombinedImuFactorExample,
              self).__init__(twist_scenarios[twist_scenario], bias, params, dt)

    def addPrior(self, i: int, graph: gtsam.NonlinearFactorGraph):
        """Add a prior on the navigation state at time `i`."""
        state = self.scenario.navState(i)
        graph.push_back(
            gtsam.PriorFactorPose3(X(i), state.pose(), self.priorNoise))
        graph.push_back(
            gtsam.PriorFactorVector(V(i), state.velocity(), self.velNoise))
        graph.push_back(
            gtsam.PriorFactorConstantBias(B(i), self.actualBias,
                                          self.biasNoise))

    def optimize(self, graph: gtsam.NonlinearFactorGraph,
                 initial: gtsam.Values):
        """Optimize using Levenberg-Marquardt optimization."""
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM("SUMMARY")
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
        result = optimizer.optimize()
        return result

    def plot(self,
             values: gtsam.Values,
             title: str = "Estimated Trajectory",
             fignum: int = POSES_FIG + 1,
             show: bool = False):
        """
        Plot poses in values.

        Args:
            values: The values object with the poses to plot.
            title: The title of the plot.
            fignum: The matplotlib figure number.
                POSES_FIG is a value from the PreintegrationExample
                which we simply increment to generate a new figure.
            show: Flag indicating whether to display the figure.
        """
        i = 0
        while values.exists(X(i)):
            pose_i = values.atPose3(X(i))
            plot_pose3(fignum, pose_i, 1)
            i += 1
        plt.title(title)

        gtsam.utils.plot.set_axes_equal(fignum)

        i = 0
        while values.exists(B(i)):
            print("Bias Value {0}".format(i), values.atConstantBias(B(i)))
            i += 1

        plt.ioff()

        if show:
            plt.show()

    def run(self,
            T: int = 12,
            compute_covariances: bool = False,
            verbose: bool = True):
        """
        Main runner.

        Args:
            T: Total trajectory time.
            compute_covariances: Flag indicating whether to compute marginal covariances.
            verbose: Flag indicating if printing should be verbose.
        """
        graph = gtsam.NonlinearFactorGraph()

        # initialize data structure for pre-integrated IMU measurements
        pim = gtsam.PreintegratedCombinedMeasurements(self.params,
                                                      self.actualBias)

        num_poses = T  # assumes 1 factor per second
        initial = gtsam.Values()

        # simulate the loop
        i = 0  # state index
        initial_state_i = self.scenario.navState(0)
        initial.insert(X(i), initial_state_i.pose())
        initial.insert(V(i), initial_state_i.velocity())
        initial.insert(B(i), self.actualBias)

        # add prior on beginning
        self.addPrior(0, graph)

        for k, t in enumerate(np.arange(0, T, self.dt)):
            # get measurements and add them to PIM
            measuredOmega = self.runner.measuredAngularVelocity(t)
            measuredAcc = self.runner.measuredSpecificForce(t)
            pim.integrateMeasurement(measuredAcc, measuredOmega, self.dt)

            # Plot IMU many times
            if k % 10 == 0:
                self.plotImu(t, measuredOmega, measuredAcc)

            if (k + 1) % int(1 / self.dt) == 0:
                # Plot every second
                self.plotGroundTruthPose(t, scale=1)
                plt.title("Ground Truth Trajectory")

                # create IMU factor every second
                factor = gtsam.CombinedImuFactor(X(i), V(i), X(i + 1),
                                                 V(i + 1), B(i), B(i + 1), pim)
                graph.push_back(factor)

                if verbose:
                    print(factor)
                    print("Predicted state at {0}:\n{1}".format(
                        t + self.dt,
                        pim.predict(initial_state_i, self.actualBias)))

                pim.resetIntegration()

                rotationNoise = gtsam.Rot3.Expmap(np.random.randn(3) * 0.1)
                translationNoise = gtsam.Point3(*np.random.randn(3) * 1)
                poseNoise = gtsam.Pose3(rotationNoise, translationNoise)

                actual_state_i = self.scenario.navState(t + self.dt)
                print("Actual state at {0}:\n{1}".format(
                    t + self.dt, actual_state_i))

                # Set initial state to current
                initial_state_i = actual_state_i

                noisy_state_i = gtsam.NavState(
                    actual_state_i.pose().compose(poseNoise),
                    actual_state_i.velocity() + np.random.randn(3) * 0.1)
                noisy_bias_i = self.actualBias + gtsam.imuBias.ConstantBias(
                    np.random.randn(3) * 0.1,
                    np.random.randn(3) * 0.1)

                initial.insert(X(i + 1), noisy_state_i.pose())
                initial.insert(V(i + 1), noisy_state_i.velocity())
                initial.insert(B(i + 1), noisy_bias_i)
                i += 1

        # add priors on end
        self.addPrior(num_poses - 1, graph)

        initial.print("Initial values:")

        result = self.optimize(graph, initial)

        result.print("Optimized values:")
        print("------------------")
        print("Initial Error =", graph.error(initial))
        print("Final Error =", graph.error(result))
        print("------------------")

        if compute_covariances:
            # Calculate and print marginal covariances
            marginals = gtsam.Marginals(graph, result)
            print("Covariance on bias:\n",
                  marginals.marginalCovariance(BIAS_KEY))
            for i in range(num_poses):
                print("Covariance on pose {}:\n{}\n".format(
                    i, marginals.marginalCovariance(X(i))))
                print("Covariance on vel {}:\n{}\n".format(
                    i, marginals.marginalCovariance(V(i))))

        self.plot(result, show=True)


if __name__ == '__main__':
    args = parse_args()

    CombinedImuFactorExample(args.twist_scenario).run(args.time,
                                                      args.compute_covariances,
                                                      args.verbose)
