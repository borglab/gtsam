"""
A script validating the ImuFactor prediction and inference.
"""

import math
import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

import gtsam
from gtsam_utils import plotPose3

class ImuFactorExample(object):

    @staticmethod
    def defaultParams(g):
        """Create default parameters with Z *up* and realistic noise parameters"""
        params = gtsam.PreintegrationParams.MakeSharedU(g)
        kGyroSigma = math.radians(0.5) / 60  # 0.5 degree ARW
        kAccelSigma = 0.1 / 60  # 10 cm VRW
        params.gyroscopeCovariance = kGyroSigma ** 2 * np.identity(3, np.float)
        params.accelerometerCovariance = kAccelSigma ** 2 * np.identity(3, np.float)
        params.integrationCovariance = 0.0000001 ** 2 * np.identity(3, np.float)
        return params

    def __init__(self):
        # setup interactive plotting
        plt.ion()

        # Setup loop scenario
        # Forward velocity 2m/s
        # Pitch up with angular velocity 6 degree/sec (negative in FLU)
        v = 2
        w = math.radians(30)
        W = np.array([0, -w, 0])
        V = np.array([v, 0, 0])
        self.scenario = gtsam.ConstantTwistScenario(W, V)
        self.dt = 0.25
        self.realTimeFactor = 10.0

        # Calculate time to do 1 loop
        self.radius = v / w
        self.timeForOneLoop = 2 * math.pi / w
        self.labels = list('xyz')
        self.colors = list('rgb')

        # Create runner
        dt = 0.1
        self.g = 10  # simple gravity constant
        self.params = self.defaultParams(self.g)
        self.runner = gtsam.ScenarioRunner(gtsam.ScenarioPointer(self.scenario), self.params, dt)
        self.estimatedBias = gtsam.ConstantBias()

    def plot(self, t, measuredOmega, measuredAcc):
        # plot angular velocity    
        omega_b = self.scenario.omega_b(t)
        plt.figure(1)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            plt.subplot(3, 1, i + 1)
            plt.scatter(t, omega_b[i], color='k', marker='.')
            plt.scatter(t, measuredOmega[i], color=color, marker='.')
            plt.xlabel(label)

        # plot acceleration in nav
        plt.figure(2)
        acceleration_n = self.scenario.acceleration_n(t)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            plt.subplot(3, 1, i + 1)
            plt.scatter(t, acceleration_n[i], color=color, marker='.')
            plt.xlabel(label)

        # plot acceleration in body
        plt.figure(3)
        acceleration_b = self.scenario.acceleration_b(t)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            plt.subplot(3, 1, i + 1)
            plt.scatter(t, acceleration_b[i], color=color, marker='.')
            plt.xlabel(label)

        # plot ground truth pose, as well as prediction from integrated IMU measurements
        actualPose = self.scenario.pose(t)
        plotPose3(4, actualPose, 1.0)
        pim = self.runner.integrate(t, self.estimatedBias, False)
        predictedNavState = self.runner.predict(pim, self.estimatedBias)
        plotPose3(4, predictedNavState.pose(), 1.0)
        ax = plt.gca()
        ax.set_xlim3d(-self.radius, self.radius)
        ax.set_ylim3d(-self.radius, self.radius)
        ax.set_zlim3d(0, self.radius * 2)

        # plot actual specific force, as well as corrupted
        plt.figure(5)
        actual = self.runner.actualSpecificForce(t)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            plt.subplot(3, 1, i + 1)
            plt.scatter(t, actual[i], color='k', marker='.')
            plt.scatter(t, measuredAcc[i], color=color, marker='.')
            plt.xlabel(label)

        plt.pause(self.dt / self.realTimeFactor)

    def run(self):
        # simulate the loop up to the top
        for t in np.arange(0, self.timeForOneLoop, self.dt):
            measuredOmega = self.runner.measuredAngularVelocity(t)
            measuredAcc = self.runner.measuredSpecificForce(t)
            self.plot(t, measuredOmega, measuredAcc)

        plt.ioff()
        plt.show()

if __name__ == '__main__':
    ImuFactorExample().run()
