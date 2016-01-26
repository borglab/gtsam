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

    def __init__(self):
        # setup interactive plotting
        plt.ion()

        # Setup loop scenario
        # Forward velocity 2m/s
        # Pitch up with angular velocity 6 degree/sec (negative in FLU)
        v = 2
        w = math.radians(6)
        W = np.array([0, -w, 0])
        V = np.array([v, 0, 0])
        self.scenario = gtsam.ConstantTwistScenario(W, V)
        self.dt = 0.5
        self.realTimeFactor = 10.0
        
        # Calculate time to do 1 loop
        self.radius = v / w
        self.timeForOneLoop = 2 * math.pi / w
        self.labels = list('xyz')
        self.colors = list('rgb')

    def plot(self, t, pose, omega_b, acceleration_n, acceleration_b):
        # plot angular velocity    
        plt.figure(1)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            plt.subplot(3, 1, i + 1)
            plt.scatter(t, omega_b[i], color=color, marker='.')
            plt.xlabel(label)
            
        # plot acceleration in nav
        plt.figure(2)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            plt.subplot(3, 1, i + 1)
            plt.scatter(t, acceleration_n[i], color=color, marker='.')
            plt.xlabel(label)
            
        # plot acceleration in body
        plt.figure(3)
        for i, (label, color) in enumerate(zip(self.labels, self.colors)):
            plt.subplot(3, 1, i + 1)
            plt.scatter(t, acceleration_b[i], color=color, marker='.')
            plt.xlabel(label)
            
        # plot ground truth
        plotPose3(4, pose, 1.0)
        ax = plt.gca()
        ax.set_xlim3d(-self.radius, self.radius)
        ax.set_ylim3d(-self.radius, self.radius)
        ax.set_zlim3d(0, self.radius * 2)

        plt.pause(self.dt / self.realTimeFactor)

    def run(self):
        # simulate the loop up to the top
        for t in np.arange(0, self.timeForOneLoop, self.dt):
            self.plot(t,
                      self.scenario.pose(t),
                      self.scenario.omega_b(t),
                      self.scenario.acceleration_n(t),
                      self.scenario.acceleration_b(t))

        plt.ioff()
        plt.show()

if __name__ == '__main__':
    ImuFactorExample().run()
