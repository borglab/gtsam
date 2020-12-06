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

import matplotlib.pyplot as plt
import numpy as np

import gtsam
from gtsam.utils import plot

def estimate_poses_given_rot(factors: gtsam.BetweenFactorPose3s,
                             rotations: gtsam.Values,
                             d: int = 3):
    """ Estimate Poses from measurements, given rotations. From SfmProblem in shonan.

    Arguments:
        factors -- data structure with many BetweenFactorPose3 factors
        rotations {Values} -- Estimated rotations

    Returns:
        Values -- Estimated Poses
    """

    I_d = np.eye(d)

    def R(j):
        return rotations.atRot3(j) if d == 3 else rotations.atRot2(j)

    def pose(R, t):
        return gtsam.Pose3(R, t) if d == 3 else gtsam.Pose2(R, t)

    graph = gtsam.GaussianFactorGraph()
    model = gtsam.noiseModel.Unit.Create(d)

    # Add a factor anchoring t_0
    graph.add(0, I_d, np.zeros((d,)), model)

    # Add a factor saying t_j - t_i = Ri*t_ij for all edges (i,j)
    for factor in factors:
        keys = factor.keys()
        i, j, Tij = keys[0], keys[1], factor.measured()
        measured = R(i).rotate(Tij.translation())
        graph.add(j, I_d, i, -I_d, measured, model)

    # Solve linear system
    translations = graph.optimize()

    # Convert to Values.
    result = gtsam.Values()
    for j in range(rotations.size()):
        tj = translations.at(j)
        result.insert(j, pose(R(j), tj))

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
        if shonan.nrUnknowns() == 0:
            raise ValueError("No 2D pose constraints found, try -d 3.")
        initial = shonan.initializeRandomly()
        rotations, _ = shonan.run(initial, 2, 10)
        factors = gtsam.parse2DFactors(input_file)
    elif args.dimension == 3:
        print("Running Shonan averaging for SO(3) on ", input_file)
        shonan = gtsam.ShonanAveraging3(input_file)
        if shonan.nrUnknowns() == 0:
            raise ValueError("No 3D pose constraints found, try -d 2.")
        initial = shonan.initializeRandomly()
        rotations, _ = shonan.run(initial, 3, 10)
        factors = gtsam.parse3DFactors(input_file)
    else:
        raise ValueError("Can only run SO(2) or SO(3) averaging")

    print("Recovering translations")
    poses = estimate_poses_given_rot(factors, rotations, args.dimension)

    print("Writing result to ", args.output_file)
    gtsam.writeG2o(gtsam.NonlinearFactorGraph(), poses, args.output_file)

    plot.plot_trajectory(1, poses, scale=0.2)
    plot.set_axes_equal(1)
    plt.show()


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
    parser.add_argument("-p", "--plot", action="store_true", default=True,
                        help="Plot result")
    run(parser.parse_args())
