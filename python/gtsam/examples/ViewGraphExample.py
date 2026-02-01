"""
  GTSAM Copyright 2010, Georgia Tech Research Corporation,
  Atlanta, Georgia 30332-0415
  All Rights Reserved
  Authors: Frank Dellaert, et al. (see THANKS for the full author list)

  See LICENSE for the license information
"""

"""
Python version of ViewGraphExample.cpp
View-graph calibration on a simulated dataset, a la Sweeney 2015
Author: Frank Dellaert
Date: October 2024
"""

import numpy as np
from gtsam.examples import SFMdata

from gtsam import (Cal3_S2, EdgeKey, FundamentalMatrix,
                   LevenbergMarquardtOptimizer, LevenbergMarquardtParams,
                   NonlinearFactorGraph, PinholeCameraCal3_S2)
from gtsam import TransferFactorFundamentalMatrix as Factor
from gtsam import Values


# Formatter function for printing keys
def formatter(key):
    edge = EdgeKey(key)
    return f"({edge.i()},{edge.j()})"


def main():
    # Define the camera calibration parameters
    cal = Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)

    # Create the set of 8 ground-truth landmarks
    points = SFMdata.createPoints()

    # Create the set of 4 ground-truth poses
    poses = SFMdata.posesOnCircle(4, 30)

    # Calculate ground truth fundamental matrices, 1 and 2 poses apart
    F1 = FundamentalMatrix(cal.K(), poses[0].between(poses[1]), cal.K())
    F2 = FundamentalMatrix(cal.K(), poses[0].between(poses[2]), cal.K())

    # Simulate measurements from each camera pose
    p = [[None for _ in range(8)] for _ in range(4)]
    for i in range(4):
        camera = PinholeCameraCal3_S2(poses[i], cal)
        for j in range(8):
            p[i][j] = camera.project(points[j])

    # Create the factor graph
    graph = NonlinearFactorGraph()

    for a in range(4):
        b = (a + 1) % 4  # Next camera
        c = (a + 2) % 4  # Camera after next

        # Vectors to collect tuples for each factor
        tuples1 = []
        tuples2 = []
        tuples3 = []

        # Collect data for the three factors
        for j in range(8):
            tuples1.append((p[a][j], p[b][j], p[c][j]))
            tuples2.append((p[a][j], p[c][j], p[b][j]))
            tuples3.append((p[c][j], p[b][j], p[a][j]))

        # Add transfer factors between views a, b, and c.
        graph.add(Factor(EdgeKey(a, c), EdgeKey(b, c), tuples1))
        graph.add(Factor(EdgeKey(a, b), EdgeKey(b, c), tuples2))
        graph.add(Factor(EdgeKey(a, c), EdgeKey(a, b), tuples3))

    # Print the factor graph
    graph.print("Factor Graph:\n", formatter)

    # Create a delta vector to perturb the ground truth
    delta = np.array([1, 2, 3, 4, 5, 6, 7]) * 1e-5

    # Create the data structure to hold the initial estimate to the solution
    initialEstimate = Values()
    for a in range(4):
        b = (a + 1) % 4  # Next camera
        c = (a + 2) % 4  # Camera after next
        initialEstimate.insert(EdgeKey(a, b).key(), F1.retract(delta))
        initialEstimate.insert(EdgeKey(a, c).key(), F2.retract(delta))

    initialEstimate.print("Initial Estimates:\n", formatter)
    graph.printErrors(initialEstimate, "Initial Errors:\n", formatter)

    # Optimize the graph and print results
    params = LevenbergMarquardtParams()
    params.setlambdaInitial(1000.0)  # Initialize lambda to a high value
    params.setVerbosityLM("SUMMARY")
    optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, params)
    result = optimizer.optimize()

    print(f"Initial error = {graph.error(initialEstimate)}")
    print(f"Final error = {graph.error(result)}")

    result.print("Final Results:\n", formatter)

    print("Ground Truth F1:\n", F1.matrix())
    print("Ground Truth F2:\n", F2.matrix())


if __name__ == "__main__":
    main()
