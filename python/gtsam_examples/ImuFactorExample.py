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
        self.priorNoise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        self.velNoise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
    
    def addPrior(self, i, graph):
        state = self.scenario.navState(i)
        graph.push_back(gtsam.PriorFactorPose3(X(i), state.pose(), self.priorNoise))
        graph.push_back(gtsam.PriorFactorVector3(V(i), state.velocity(), self.velNoise))
    
    def run(self):
        graph = gtsam.NonlinearFactorGraph()

        i = 0  # state index
        
        # initialize data structure for pre-integrated IMU measurements 
        pim = gtsam.PreintegratedImuMeasurements(self.params, self.actualBias)
        
        # simulate the loop
        T = 12
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
            if k % 100 == 0:
                self.plotGroundTruthPose(t)
            
            # create IMU factor every second
            if (k + 1) % 100 == 0:
                factor = gtsam.ImuFactor(X(i), V(i), X(i + 1), V(i + 1), BIAS_KEY, pim)
                graph.push_back(factor)
                H1 = gtsam.OptionalJacobian9()
                H2 = gtsam.OptionalJacobian96()
                predicted_state_j = pim.predict(actual_state_i, self.actualBias, H1, H2)
                error = pim.computeError(actual_state_i, predicted_state_j, self.actualBias, H1, H1, H2)
                print("error={}, norm ={}".format(error, np.linalg.norm(error)))
                pim.resetIntegration()
                actual_state_i = self.scenario.navState(t + self.dt)
                i += 1

        # add priors on beginning and end
        num_poses = i + 1
        self.addPrior(0, graph)
        self.addPrior(num_poses - 1, graph)
        
#         graph.print("\Graph:\n")

        initial = gtsam.Values()
        initial.insert(BIAS_KEY, self.actualBias)
        for i in range(num_poses):
            state_i = self.scenario.navState(float(i))
            plotPose3(POSES_FIG, state_i.pose(), 0.9)
            initial.insert(X(i), state_i.pose())
            initial.insert(V(i), state_i.velocity())
        
        for idx in range(num_poses - 1):
            ff = gtsam.getNonlinearFactor(graph, idx)
            print(ff.error(initial))

        # optimize using Levenberg-Marquardt optimization
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM("SUMMARY")
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
        result = optimizer.optimize()
        result.print("\Result:\n")

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
