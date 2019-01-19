"""
 * @file Pose3SLAMExample_initializePose3.cpp
 * @brief A 3D Pose SLAM example that reads input from g2o, and initializes the Pose3 using InitializePose3
 * Syntax for the script is ./Pose3SLAMExample_initializePose3 input.g2o output.g2o
 * @date Jan 17, 2019 
 * @author Vikrant Shah based on CPP example by Luca Carlone
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import math

import numpy as np

import gtsam

import sys

def vector6(x, y, z, a, b, c):
    """Create 6d double numpy array."""
    return np.array([x, y, z, a, b, c], dtype=np.float)

if len(sys.argv) < 2:
    g2oFile = gtsam.findExampleDataFile("pose3example.txt")
else:
    g2oFile = str(sys.argv[1])
is3D = True
graph, initial = gtsam.readG2o(g2oFile,is3D)

# Add Prior on the first key
priorModel = gtsam.noiseModel_Diagonal.Variances(vector6(1e-6, 1e-6, 1e-6, 
                                                         1e-4, 1e-4, 1e-4))

print("Adding prior to g2o file ")
graphWithPrior = graph
firstKey = initial.keys().at(0)
graphWithPrior.add(gtsam.PriorFactorPose3(firstKey, gtsam.Pose3(), priorModel))

params = gtsam.GaussNewtonParams()
params.setVerbosity("Termination") # this will show info about stopping conditions
optimizer = gtsam.GaussNewtonOptimizer(graphWithPrior, initial, params)
result = optimizer.optimize()
print("Optimization complete")

print("initial error = ",graphWithPrior.error(initial))
print("final error = ",graphWithPrior.error(result))

if len(sys.argv) < 3:
    print("Final Result:\n{}".format(result))
else:
    outputFile = sys.argv[2]
    print("Writing results to file: ", outputFile)
    graphNoKernel, initial2 = gtsam.readG2o(g2oFile,is3D)
    gtsam.writeG2o(graphNoKernel, result, outputFile)
    print ("Done!")
