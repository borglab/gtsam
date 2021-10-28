"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)
See LICENSE for the license information

A 2D Pose SLAM example that reads input from g2o, and solve the Pose2 problem
using LAGO (Linear Approximation for Graph Optimization).
Output is written to a file, in g2o format

Reference:
L. Carlone, R. Aragues, J. Castellanos, and B. Bona, A fast and accurate
approximation for planar pose graph optimization, IJRR, 2014.

L. Carlone, R. Aragues, J.A. Castellanos, and B. Bona, A linear approximation
for graph-based simultaneous localization and mapping, RSS, 2011.

Author: Luca Carlone (C++), John Lambert (Python)
"""

import argparse
from argparse import Namespace

import numpy as np

import gtsam
from gtsam import Point3, Pose2, PriorFactorPose2, Values


def run(args: Namespace) -> None:
    """Run LAGO on input data stored in g2o file."""
    g2oFile = gtsam.findExampleDataFile("noisyToyGraph.txt") if args.input is None else args.input

    graph = gtsam.NonlinearFactorGraph()
    graph, initial = gtsam.readG2o(g2oFile)

    # Add prior on the pose having index (key) = 0
    priorModel = gtsam.noiseModel.Diagonal.Variances(Point3(1e-6, 1e-6, 1e-8))
    graph.add(PriorFactorPose2(0, Pose2(), priorModel))
    print(graph)

    print("Computing LAGO estimate")
    estimateLago: Values = gtsam.lago.initialize(graph)
    print("done!")

    if args.output is None:
        estimateLago.print("estimateLago")
    else:
        outputFile = args.output
        print("Writing results to file: ", outputFile)
        graphNoKernel = gtsam.NonlinearFactorGraph()
        graphNoKernel, initial2 = gtsam.readG2o(g2oFile)
        gtsam.writeG2o(graphNoKernel, estimateLago, outputFile)
        print("Done! ")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A 2D Pose SLAM example that reads input from g2o, "
        "converts it to a factor graph and does the optimization. "
        "Output is written on a file, in g2o format"
    )
    parser.add_argument("-i", "--input", help="input file g2o format")
    parser.add_argument("-o", "--output", help="the path to the output file with optimized graph")
    args = parser.parse_args()
    run(args)
