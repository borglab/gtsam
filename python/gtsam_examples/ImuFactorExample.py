"""
A script validating the ImuFactor inference.
"""

from __future__ import print_function
import math
import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

import gtsam
from gtsam_utils import plotPose3
from PreintegrationExample import PreintegrationExample, POSES_FIG

# shorthand symbols:
BIAS_KEY = int(gtsam.Symbol('b', 0))
V = lambda j: int(gtsam.Symbol('v', j))
X = lambda i: int(gtsam.Symbol('x', i))

class ImuFactorExample(PreintegrationExample):

    def run(self):
        graph = gtsam.NonlinearFactorGraph()
        for i in [0, 12]:
            priorNoise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
            graph.push_back(gtsam.PriorFactorPose3(X(i), gtsam.Pose3(), priorNoise))
            velNoise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
            graph.push_back(gtsam.PriorFactorVector3(V(i), np.array([2, 0, 0]), velNoise))
        
        i = 0  # state index
        
        # initialize data structure for pre-integrated IMU measurements 
        pim = gtsam.PreintegratedImuMeasurements(self.params, self.actualBias)
        
        # simulate the loop
        T = self.timeForOneLoop
        for k, t in enumerate(np.arange(0, T, self.dt)):
            # get measurements and add them to PIM
            measuredOmega = self.runner.measuredAngularVelocity(t)
            measuredAcc = self.runner.measuredSpecificForce(t)
            pim.integrateMeasurement(measuredAcc, measuredOmega, self.dt)
            
            # Plot every second
            if k % 100 == 0:
                self.plotImu(t, measuredOmega, measuredAcc)
                self.plotGroundTruthPose(t)
            
            # create factor every second
            if (k + 1) % 100 == 0:
                factor = gtsam.ImuFactor(X(i), V(i), X(i + 1), V(i + 1), BIAS_KEY, pim)
                graph.push_back(factor)
                pim.resetIntegration()
                i += 1

        graph.print()
        num_poses = i + 1

        initial = gtsam.Values()
        initial.insert(BIAS_KEY, gtsam.ConstantBias())
        for i in range(num_poses):
            initial.insert(X(i), gtsam.Pose3())
            initial.insert(V(i), np.zeros(3, np.float))
        
        # optimize using Levenberg-Marquardt optimization
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM("SUMMARY")
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
        result = optimizer.optimize()
        # result.print("\Result:\n")
        print(self.actualBias)

        # Plot cameras
        i = 0
        while result.exists(X(i)):
            pose_i = result.pose3_at(X(i))
            plotPose3(POSES_FIG, pose_i, 0.1)
            i += 1
            
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    ImuFactorExample().run()
