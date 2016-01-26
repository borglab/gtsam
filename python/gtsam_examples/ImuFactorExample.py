"""
A script validating the ImuFactor prediction and inference.
"""

import math
import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

import gtsam

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

        # Calculate time to do 1 loop
        self.T = 2 * math.pi / w

    def plot(self, t, pose):
        # plot IMU    
        plt.figure(1)
        times = np.arange(0, 10, 0.1)
        shape = len(times), 1
        labels = list('xyz')
        colors = list('rgb')
        plt.clf()
        for i, (label, color) in enumerate(zip(labels, colors)):
            plt.subplot(3, 1, i + 1)
            imu = np.random.randn(len(times), 1)
            plt.plot(times, imu, color=color)
#             plt.axis([tmin, tmax, min,max])
            plt.xlabel(label)
            
        # plot ground truth
        fig = plt.figure(2)
        ax = fig.gca(projection='3d')
        p = pose.translation()
        ax.scatter(p.x(), p.y(), p.z())

        plt.pause(0.1)

    def run(self):
        for t in np.arange(0, self.T / 2, 1):
            pose = self.scenario.pose(t)
            self.plot(t, pose)

        plt.ioff()
        plt.show()

if __name__ == '__main__':
    ImuFactorExample().run()
