"""
A script validating the ImuFactor inference.
"""

import math
import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

import gtsam
from gtsam_utils import plotPose3
from PreintegrationExample import PreintegrationExample

class ImuFactorExample(PreintegrationExample):

    def run(self):
        # simulate the loop up to the top
        T = self.timeForOneLoop
        pim = gtsam.PreintegratedImuMeasurements(self.params, self.actualBias)
        for i, t in enumerate(np.arange(0, T, self.dt)):
            measuredOmega = self.runner.measuredAngularVelocity(t)
            measuredAcc = self.runner.measuredSpecificForce(t)
            if i % 25 == 0:
                self.plotImu(t, measuredOmega, measuredAcc)
                self.plotGroundTruthPose(t)

        plt.ioff()
        plt.show()

if __name__ == '__main__':
    ImuFactorExample().run()
