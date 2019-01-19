"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

A 2D Pose SLAM example that reads input from g2o, converts it to a factor graph and does the optimization. Output is written on a file, in g2o format 

Syntax for the script is "python ./Pose2SLAMExample_g2o.py input.g2o output.g2o"
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function
import math
import numpy as np
import gtsam
import sys


def vector3(x, y, z):
    """Create 3d double numpy array."""
    return np.array([x, y, z], dtype=np.float)

kernelType = "none"
maxIterations = 100 # default
g2oFile = gtsam.findExampleDataFile("noisyToyGraph.txt") # default

if len(sys.argv) > 1:
    g2ofile = str(sys.argv[1])
    print ("Input file: ",g2ofile)
if len(sys.argv) > 3:
    maxIterations = int(sys.argv[3])
    print("Sepficied max iterations: ", maxIterations)
if len(sys.argv) > 4:
    kernelType = sys.argv[4]

is3D = False # readG2o 3d parameter not available at time of this writing

if kernelType is "none":
    graph, initial = gtsam.readG2o(g2oFile)
if kernelType is "huber":
    print("Using robust kernel: huber - NOT CURRENTLY IMPLEMENTED IN PYTHON")
    #graph, initial = gtsam.readG2o(g2oFile,is3D, KernelFunctionTypeHUBER)
if kernelType is "tukey":
    print("Using robust kernel: tukey - NOT CURRENTLY IMPLEMENTED IN PYTHON")
    #graph, initial = gtsam.readG2o(g2oFile,is3D, KernelFunctionTypeTUKEY)

# Add prior on the pose having index (key) = 0
graphWithPrior = graph
priorModel = gtsam.noiseModel_Diagonal.Variances(vector3(1e-6, 1e-6, 1e-8))
graphWithPrior.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), priorModel))
print("Adding prior on pose 0 ")

print("\nFactor Graph:\n{}".format(graph)) 

print("\nInitial Estimate:\n{}".format(initial))

params = gtsam.GaussNewtonParams()
params.setVerbosity("Termination")

if (sys.argv > 3):
    params.setMaxIterations(maxIterations)
    print("User setting: required to perform maximum ", maxIterations," iterations ")

#parameters.setRelativeErrorTol(1e-5)

# Create the optimizer ...
optimizer = gtsam.GaussNewtonOptimizer(graphWithPrior, initial, params)
# ... and optimize
result = optimizer.optimize()

print("Optimization complete")
print("initial error = ", graph.error(initial))
print("final error = ", graph.error(result))

if len(sys.argv) < 3:
    print("Final Result:\n{}".format(result))
else:
    outputFile = sys.argv[2]
    print("Writing results to file: ", outputFile)
    graphNoKernel, initial2 = gtsam.readG2o(g2oFile)
    gtsam.writeG2o(graphNoKernel, result, outputFile)
    print ("Done!")
