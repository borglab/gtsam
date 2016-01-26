"""
A script validating the ImuFactor prediction and inference.
"""

from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np

class ImuFactorExample(object):

    def __init__(self):
        plt.figure(1)
        plt.ion()

    def plot(self):
        times = np.arange(0, 10, 0.1)
        shape = len(times), 1
        labels = list('xyz')
        colors = list('rgb')
        plt.clf()
        for row, (label, color) in enumerate(zip(labels, colors)):
            plt.subplot(3, 1, row)
            imu = np.random.randn(len(times), 1)
            plt.plot(times, imu, color=color)
#             plt.axis([tmin, tmax, min,max])
            plt.xlabel(label)
        plt.pause(0.1)

    def run(self):
        for i in range(100):
            self.plot()

if __name__ == '__main__':
    ImuFactorExample().run()
