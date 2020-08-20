"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Shonan Rotation Averaging CLI reads a *pose* graph, extracts the
rotation constraints, and runs the Shonan algorithm.

Author: Frank Dellaert
Date: August 2020
"""
# pylint: disable=invalid-name, E1101

import argparse
import numpy as np

import gtsam


def estimate_poses_given_rot(factors: gtsam.BetweenFactorPose3s, rotations: gtsam.Values, d: int = 3):
    """ Estimate Poses from measurements, given rotations. From SfmProblem in shonan.

    Arguments:
        factors -- data structure with many BetweenFactorPose3 factors
        rotations {Values} -- Estimated rotations

    Returns:
        Values -- Estimated Poses
    """

    def R(j):
        return rotations.atRot3(j) if d == 3 else rotations.atRot2(j)

    graph = gtsam.GaussianFactorGraph()
    model = gtsam.noiseModel.Unit.Create(3)

    # Add a factor anchoring t_0
    I3 = np.eye(3)
    graph.add(0, I3, np.zeros((3,)), model)

    # Add a factor saying t_j - t_i = Ri*t_ij for all edges (i,j)
    for factor in factors:
        keys = factor.keys()
        i, j, Tij = keys[0], keys[1], factor.measured()
        measured = R(i).rotate(Tij.translation())
        graph.add(j, I3, i, -I3, measured.vector(), model)

    # Solve linear system
    translations = graph.optimize()

    # Convert to Values.
    result = gtsam.Values()
    for j in range(rotations.size()):
        tj = gtsam.Point3(translations.at(j))
        result.insert(j, gtsam.Pose3(R(j), tj))

    return result

def run(args):
    """Run Shonan averaging and then recover translations linearly before saving result."""

    # Get input file
    if args.input_file:
        input_file = args.input_file
    else:
        if args.named_dataset == "":
            raise ValueError(
                "You must either specify a named dataset or an input file")
        input_file = gtsam.findExampleDataFile(args.named_dataset)

    if args.dimension == 2:
        print("Running Shonan averaging for SO(2) on ", input_file)
        shonan = gtsam.ShonanAveraging2(input_file)
        initial = shonan.initializeRandomly()
        rotations, _ = shonan.run(initial, 2, 10)
    elif args.dimension == 3:
        print("Running Shonan averaging for SO(3) on ", input_file)
        shonan = gtsam.ShonanAveraging3(input_file)
        initial = shonan.initializeRandomly()
        rotations, _ = shonan.run(initial, 3, 10)
    else:
        raise ValueError("Can only run SO(2) or SO(3) averaging")

    factors = gtsam.parse3DFactors(input_file)
    poses = estimate_poses_given_rot(factors, rotations, args.dimension)
    print("Writing result to ", args.output_file)
    gtsam.writeG2o(gtsam.NonlinearFactorGraph(), poses, args.output_file)
    print(poses)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--named_dataset', type=str, default="pose3example-grid",
                        help='Find and read frome example dataset file')
    parser.add_argument('-i', '--input_file', type=str, default="",
                        help='Read pose constraints graph from the specified file')
    parser.add_argument('-o', '--output_file', type=str, default="shonan.g2o",
                        help='Write solution to the specified file')
    parser.add_argument('-d', '--dimension', type=int, default=3,
                        help='Optimize over 2D or 3D rotations')
    run(parser.parse_args())

