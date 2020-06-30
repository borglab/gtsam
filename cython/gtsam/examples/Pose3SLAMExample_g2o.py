"""
 * @file Pose3SLAMExample_initializePose3.cpp
 * @brief A 3D Pose SLAM example that reads input from g2o, and initializes the
 *  Pose3 using InitializePose3
 * @date Jan 17, 2019
 * @author Vikrant Shah based on CPP example by Luca Carlone
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import gtsam
from gtsam.utils import plot


def vector6(x, y, z, a, b, c):
    """Create 6d double numpy array."""
    return np.array([x, y, z, a, b, c], dtype=np.float)


parser = argparse.ArgumentParser(
    description="A 3D Pose SLAM example that reads input from g2o, and "
                "initializes Pose3")
parser.add_argument('-i', '--input', help='input file g2o format')
parser.add_argument('-o', '--output',
                    help="the path to the output file with optimized graph")
parser.add_argument("-p", "--plot", action="store_true",
                    help="Flag to plot results")
args = parser.parse_args()

g2oFile = gtsam.findExampleDataFile("pose3example.txt") if args.input is None \
    else args.input

is3D = True
graph, initial = gtsam.readG2o(g2oFile, is3D)

# Add Prior on the first key
priorModel = gtsam.noiseModel_Diagonal.Variances(vector6(1e-6, 1e-6, 1e-6,
                                                         1e-4, 1e-4, 1e-4))

print("Adding prior to g2o file ")
firstKey = initial.keys().at(0)
graph.add(gtsam.PriorFactorPose3(firstKey, gtsam.Pose3(), priorModel))

params = gtsam.GaussNewtonParams()
params.setVerbosity("Termination")  # this will show info about stopping conds
optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
result = optimizer.optimize()
print("Optimization complete")

print("initial error = ", graph.error(initial))
print("final error = ", graph.error(result))

if args.output is None:
    print("Final Result:\n{}".format(result))
else:
    outputFile = args.output
    print("Writing results to file: ", outputFile)
    graphNoKernel, _ = gtsam.readG2o(g2oFile, is3D)
    gtsam.writeG2o(graphNoKernel, result, outputFile)
    print ("Done!")

if args.plot:
    resultPoses = gtsam.utilities_allPose3s(result)
    for i in range(resultPoses.size()):
        plot.plot_pose3(1, resultPoses.atPose3(i))
    plt.show()
