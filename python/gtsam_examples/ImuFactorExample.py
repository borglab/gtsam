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

    def __init__(self):
        self.velocity = np.array([2, 0, 0])
        forward_twist = (np.zeros(3), self.velocity)
        loop_twist = (np.array([0, -math.radians(30), 0]), self.velocity)
        super(ImuFactorExample, self).__init__(loop_twist)
    
    def run(self):
        graph = gtsam.NonlinearFactorGraph()

        i = 0  # state index
        
        # initialize data structure for pre-integrated IMU measurements 
        pim = gtsam.PreintegratedImuMeasurements(self.params, self.actualBias)
        
        # simulate the loop
        T = 3
        state = self.scenario.navState(0)
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
                H1 = gtsam.OptionalJacobian9()
                H2 = gtsam.OptionalJacobian96()
                print(pim)
                predicted = pim.predict(state, self.actualBias, H1, H2)
                pim.resetIntegration()
                state = self.scenario.navState(t + self.dt)
                print("predicted.{}\nstate.{}".format(predicted, state))
                i += 1

        # add priors on beginning and end
        num_poses = i + 1
        priorNoise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        velNoise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        for i, pose in [(0, self.scenario.pose(0)), (num_poses - 1, self.scenario.pose(T))]:
            graph.push_back(gtsam.PriorFactorPose3(X(i), pose, priorNoise))
            graph.push_back(gtsam.PriorFactorVector3(V(i), self.velocity, velNoise))
        
#         graph.print("\Graph:\n")

        initial = gtsam.Values()
        initial.insert(BIAS_KEY, self.actualBias)
        for i in range(num_poses):
            initial.insert(X(i), self.scenario.pose(float(i)))
            initial.insert(V(i), self.velocity)
        
        # optimize using Levenberg-Marquardt optimization
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM("SUMMARY")
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
        result = optimizer.optimize()
#         result.print("\Result:\n")

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
