"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Track a moving object "Time of Arrival" measurements at 4 microphones.
Author: Frank Dellaert
"""
# pylint: disable=invalid-name, no-name-in-module

from gtsam import (LevenbergMarquardtOptimizer, LevenbergMarquardtParams,
                   NonlinearFactorGraph, Point3, Values, noiseModel_Isotropic)
from gtsam_unstable import Event, TimeOfArrival, TOAFactor

# units
MS = 1e-3
CM = 1e-2

# Instantiate functor with speed of sound value
TIME_OF_ARRIVAL = TimeOfArrival(330)


def define_microphones():
    """Create microphones."""
    height = 0.5
    microphones = []
    microphones.append(Point3(0, 0, height))
    microphones.append(Point3(403 * CM, 0, height))
    microphones.append(Point3(403 * CM, 403 * CM, height))
    microphones.append(Point3(0, 403 * CM, 2 * height))
    return microphones


def create_trajectory(n):
    """Create ground truth trajectory."""
    trajectory = []
    timeOfEvent = 10
    # simulate emitting a sound every second while moving on straight line
    for key in range(n):
        trajectory.append(
            Event(timeOfEvent, 245 * CM + key * 1.0, 201.5 * CM, (212 - 45) * CM))
        timeOfEvent += 1

    return trajectory


def simulate_one_toa(microphones, event):
    """Simulate time-of-arrival measurements for a single event."""
    return [TIME_OF_ARRIVAL.measure(event, microphones[i])
            for i in range(len(microphones))]


def simulate_toa(microphones, trajectory):
    """Simulate time-of-arrival measurements for an entire trajectory."""
    return [simulate_one_toa(microphones, event)
            for event in trajectory]


def create_graph(microphones, simulatedTOA):
    """Create factor graph."""
    graph = NonlinearFactorGraph()

    # Create a noise model for the TOA error
    model = noiseModel_Isotropic.Sigma(1, 0.5 * MS)

    K = len(microphones)
    key = 0
    for toa in simulatedTOA:
        for i in range(K):
            factor = TOAFactor(key, microphones[i], toa[i], model)
            graph.push_back(factor)
        key += 1

    return graph


def create_initial_estimate(n):
    """Create initial estimate for n events."""
    initial = Values()
    zero = Event()
    for key in range(n):
        TOAFactor.InsertEvent(key, zero, initial)
    return initial


def toa_example():
    """Run example with 4 microphones and 5 events in a straight line."""
    # Create microphones
    microphones = define_microphones()
    K = len(microphones)
    for i in range(K):
        print("mic {} = {}".format(i, microphones[i]))

    # Create a ground truth trajectory
    n = 5
    groundTruth = create_trajectory(n)
    for event in groundTruth:
        print(event)

    # Simulate time-of-arrival measurements
    simulatedTOA = simulate_toa(microphones, groundTruth)
    for key in range(n):
        for i in range(K):
            print("z_{}{} = {} ms".format(key, i, simulatedTOA[key][i] / MS))

    # create factor graph
    graph = create_graph(microphones, simulatedTOA)
    print(graph.at(0))

    # Create initial estimate
    initial_estimate = create_initial_estimate(n)
    print(initial_estimate)

    # Optimize using Levenberg-Marquardt optimization.
    params = LevenbergMarquardtParams()
    params.setAbsoluteErrorTol(1e-10)
    params.setVerbosityLM("SUMMARY")
    optimizer = LevenbergMarquardtOptimizer(graph, initial_estimate, params)
    result = optimizer.optimize()
    print("Final Result:\n", result)


if __name__ == '__main__':
    toa_example()
    print("Example complete")
