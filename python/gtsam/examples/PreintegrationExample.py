"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

A script validating the Preintegration of IMU measurements.

Authors: Frank Dellaert, Varun Agrawal.
"""

# pylint: disable=invalid-name,unused-import,wrong-import-order

from typing import Optional, Sequence

import gtsam
import matplotlib.pyplot as plt
import numpy as np
from gtsam.utils.plot import plot_pose3
from mpl_toolkits.mplot3d import Axes3D

IMU_FIG = 1
POSES_FIG = 2
GRAVITY = 10


class PreintegrationExample:
    """Base class for all preintegration examples."""
    @staticmethod
    def defaultParams(g: float):
        """Create default parameters with Z *up* and realistic noise parameters"""
        params = gtsam.PreintegrationParams.MakeSharedU(g)
        kGyroSigma = np.radians(0.5) / 60  # 0.5 degree ARW
        kAccelSigma = 0.1 / 60  # 10 cm VRW
        params.setGyroscopeCovariance(kGyroSigma**2 * np.identity(3, float))
        params.setAccelerometerCovariance(kAccelSigma**2 *
                                          np.identity(3, float))
        params.setIntegrationCovariance(0.0000001**2 * np.identity(3, float))
        return params

    def __init__(self,
                 twist: Optional[np.ndarray] = None,
                 bias: Optional[gtsam.imuBias.ConstantBias] = None,
                 params: Optional[gtsam.PreintegrationParams] = None,
                 dt: float = 1e-2):
        """Initialize with given twist, a pair(angularVelocityVector, velocityVector)."""

        # setup interactive plotting
        plt.ion()

        # Setup loop as default scenario
        if twist is not None:
            (W, V) = twist
        else:
            # default = loop with forward velocity 2m/s, while pitching up
            # with angular velocity 30 degree/sec (negative in FLU)
            W = np.array([0, -np.radians(30), 0])
            V = np.array([2, 0, 0])

        self.scenario = gtsam.ConstantTwistScenario(W, V)
        self.dt = dt

        self.maxDim = 5
        self.labels = list('xyz')
        self.colors = list('rgb')

        if params:
            self.params = params
        else:
            # Default params with simple gravity constant
            self.params = self.defaultParams(g=GRAVITY)

        if bias is not None:
            self.actualBias = bias
        else:
            accBias = np.array([0, 0.1, 0])
            gyroBias = np.array([0, 0, 0])
            self.actualBias = gtsam.imuBias.ConstantBias(accBias, gyroBias)

        # Create runner
        self.runner = gtsam.ScenarioRunner(self.scenario, self.params, self.dt,
                                           self.actualBias)

        fig, self.axes = plt.subplots(4, 3)
        fig.set_tight_layout(True)

    def plotImu(self, t: float, measuredOmega: Sequence,
                measuredAcc: Sequence):
        """
        Plot IMU measurements.
        Args:
            t: The time at which the measurement was recoreded.
            measuredOmega: Measured angular velocity.
            measuredAcc: Measured linear acceleration.
        """
        plt.figure(IMU_FIG)

        # plot angular velocity
        omega_b = self.scenario.omega_b(t)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            ax = self.axes[0][i]
            ax.scatter(t, omega_b[i], color='k', marker='.')
            ax.scatter(t, measuredOmega[i], color=color, marker='.')
            ax.set_xlabel('angular velocity ' + label)

        # plot acceleration in nav
        acceleration_n = self.scenario.acceleration_n(t)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            ax = self.axes[1][i]
            ax.scatter(t, acceleration_n[i], color=color, marker='.')
            ax.set_xlabel('acceleration in nav ' + label)

        # plot acceleration in body
        acceleration_b = self.scenario.acceleration_b(t)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            ax = self.axes[2][i]
            ax.scatter(t, acceleration_b[i], color=color, marker='.')
            ax.set_xlabel('acceleration in body ' + label)

        # plot actual specific force, as well as corrupted
        actual = self.runner.actualSpecificForce(t)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            ax = self.axes[3][i]
            ax.scatter(t, actual[i], color='k', marker='.')
            ax.scatter(t, measuredAcc[i], color=color, marker='.')
            ax.set_xlabel('specific force ' + label)

    def plotGroundTruthPose(self,
                            t: float,
                            scale: float = 0.3,
                            time_interval: float = 0.01):
        """
        Plot ground truth pose, as well as prediction from integrated IMU measurements.
        Args:
            t: Time at which the pose was obtained.
            scale: The scaling factor for the pose axes.
            time_interval: The time to wait before showing the plot.
        """
        actualPose = self.scenario.pose(t)
        plot_pose3(POSES_FIG, actualPose, scale)
        translation = actualPose.translation()
        self.maxDim = max([max(np.abs(translation)), self.maxDim])
        ax = plt.gca()
        ax.set_xlim3d(-self.maxDim, self.maxDim)
        ax.set_ylim3d(-self.maxDim, self.maxDim)
        ax.set_zlim3d(-self.maxDim, self.maxDim)

        plt.pause(time_interval)

    def run(self, T: int = 12):
        """Simulate the loop."""
        for i, t in enumerate(np.arange(0, T, self.dt)):
            measuredOmega = self.runner.measuredAngularVelocity(t)
            measuredAcc = self.runner.measuredSpecificForce(t)
            if i % 25 == 0:
                self.plotImu(t, measuredOmega, measuredAcc)
                self.plotGroundTruthPose(t)
                pim = self.runner.integrate(t, self.actualBias, True)
                predictedNavState = self.runner.predict(pim, self.actualBias)
                plot_pose3(POSES_FIG, predictedNavState.pose(), 0.1)

        plt.ioff()
        plt.show()


if __name__ == '__main__':
    PreintegrationExample().run()
