"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

A 2D Pose SLAM example that reads input from g2o, converts it to a factor graph
and does the optimization. Output is written on a file, in g2o format
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function
import argparse
import math
import numpy as np
import matplotlib.pyplot as plt

import gtsam
from gtsam.utils import plot


def vector3(x, y, z):
    """Create 3d double numpy array."""
    return np.array([x, y, z], dtype=np.float)


parser = argparse.ArgumentParser(
    description="A 2D Pose SLAM example that reads input from g2o, "
                "converts it to a factor graph and does the optimization. "
                "Output is written on a file, in g2o format")
parser.add_argument('-i', '--input', help='input file g2o format')
parser.add_argument('-o', '--output',
                    help="the path to the output file with optimized graph")
parser.add_argument('-m', '--maxiter', type=int,
                    help="maximum number of iterations for optimizer")
parser.add_argument('-k', '--kernel', choices=['none', 'huber', 'tukey'],
                    default="none", help="Type of kernel used")
parser.add_argument("-p", "--plot", action="store_true",
                    help="Flag to plot results")
args = parser.parse_args()

g2oFile = gtsam.findExampleDataFile("noisyToyGraph.txt") if args.input is None\
    else args.input

maxIterations = 100 if args.maxiter is None else args.maxiter

is3D = False

graph, initial = gtsam.readG2o(g2oFile, is3D)

assert args.kernel == "none", "Supplied kernel type is not yet implemented"

# Add prior on the pose having index (key) = 0
priorModel = gtsam.noiseModel_Diagonal.Variances(vector3(1e-6, 1e-6, 1e-8))
graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), priorModel))

params = gtsam.GaussNewtonParams()
params.setVerbosity("Termination")
params.setMaxIterations(maxIterations)
# parameters.setRelativeErrorTol(1e-5)
# Create the optimizer ...
optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
# ... and optimize
result = optimizer.optimize()

print("Optimization complete")
print("initial error = ", graph.error(initial))
print("final error = ", graph.error(result))


if args.output is None:
    print("\nFactor Graph:\n{}".format(graph))
    print("\nInitial Estimate:\n{}".format(initial))
    print("Final Result:\n{}".format(result))
else:
    outputFile = args.output
    print("Writing results to file: ", outputFile)
    graphNoKernel, _ = gtsam.readG2o(g2oFile, is3D)
    gtsam.writeG2o(graphNoKernel, result, outputFile)
    print ("Done!")

if args.plot:
    resultPoses = gtsam.utilities_extractPose2(result)
    for i in range(resultPoses.shape[0]):
        plot.plot_pose2(1, gtsam.Pose2(resultPoses[i, :]))
    plt.show()
