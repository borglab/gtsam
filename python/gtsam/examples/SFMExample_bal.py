"""
  GTSAM Copyright 2010, Georgia Tech Research Corporation,
  Atlanta, Georgia 30332-0415
  All Rights Reserved
  Authors: Frank Dellaert, et al. (see THANKS for the full author list)

  See LICENSE for the license information

  Solve a structure-from-motion problem from a "Bundle Adjustment in the Large" file
  Author: Frank Dellaert (Python: Akshay Krishnan, John Lambert, Varun Agrawal)
"""

import argparse
import logging
import sys

import gtsam
from gtsam import (GeneralSFMFactorCal3Bundler, SfmData,
                   PriorFactorPinholeCameraCal3Bundler, PriorFactorPoint3)
from gtsam.symbol_shorthand import P  # type: ignore
from gtsam.utils import plot  # type: ignore
from matplotlib import pyplot as plt

logging.basicConfig(stream=sys.stdout, level=logging.INFO)

DEFAULT_BAL_DATASET = "dubrovnik-3-7-pre"


def plot_scene(scene_data: SfmData, result: gtsam.Values) -> None:
    """Plot the SFM results."""
    plot_vals = gtsam.Values()
    for i in range(scene_data.numberCameras()):
        plot_vals.insert(i, result.atPinholeCameraCal3Bundler(i).pose())
    for j in range(scene_data.numberTracks()):
        plot_vals.insert(P(j), result.atPoint3(P(j)))

    plot.plot_3d_points(0, plot_vals, linespec="g.")
    plot.plot_trajectory(0, plot_vals, title="SFM results")

    plt.show()


def run(args: argparse.Namespace) -> None:
    """ Run LM optimization with BAL input data and report resulting error """
    input_file = args.input_file

    # Load the SfM data from file
    scene_data = SfmData.FromBalFile(input_file)
    logging.info("read %d tracks on %d cameras\n", scene_data.numberTracks(),
                 scene_data.numberCameras())

    # Create a factor graph
    graph = gtsam.NonlinearFactorGraph()

    # We share *one* noiseModel between all projection factors
    noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0)  # one pixel in u and v

    # Add measurements to the factor graph
    for j in range(scene_data.numberTracks()):
        track = scene_data.track(j)  # SfmTrack
        # retrieve the SfmMeasurement objects
        for m_idx in range(track.numberMeasurements()):
            # i represents the camera index, and uv is the 2d measurement
            i, uv = track.measurement(m_idx)
            # note use of shorthand symbols C and P
            graph.add(GeneralSFMFactorCal3Bundler(uv, noise, i, P(j)))

    # Add a prior on pose x1. This indirectly specifies where the origin is.
    graph.push_back(
        PriorFactorPinholeCameraCal3Bundler(
            0, scene_data.camera(0),
            gtsam.noiseModel.Isotropic.Sigma(9, 0.1)))
    # Also add a prior on the position of the first landmark to fix the scale
    graph.push_back(
        PriorFactorPoint3(P(0),
                          scene_data.track(0).point3(),
                          gtsam.noiseModel.Isotropic.Sigma(3, 0.1)))

    # Create initial estimate
    initial = gtsam.Values()

    i = 0
    # add each PinholeCameraCal3Bundler
    for i in range(scene_data.numberCameras()):
        camera = scene_data.camera(i)
        initial.insert(i, camera)
        i += 1

    # add each SfmTrack
    for j in range(scene_data.numberTracks()):
        track = scene_data.track(j)
        initial.insert(P(j), track.point3())

    # Optimize the graph and print results
    try:
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM("ERROR")
        lm = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
        result = lm.optimize()
    except RuntimeError:
        logging.exception("LM Optimization failed")
        return

    # Error drops from ~2764.22 to ~0.046
    logging.info("initial error: %f", graph.error(initial))
    logging.info("final error: %f", graph.error(result))

    plot_scene(scene_data, result)


def main() -> None:
    """Main runner."""
    parser = argparse.ArgumentParser()
    parser.add_argument('-i',
                        '--input_file',
                        type=str,
                        default=gtsam.findExampleDataFile(DEFAULT_BAL_DATASET),
                        help="""Read SFM data from the specified BAL file.
        The data format is described here: https://grail.cs.washington.edu/projects/bal/.
        BAL files contain (nrPoses, nrPoints, nrObservations), followed by (i,j,u,v) tuples,
        then (wx,wy,wz,tx,ty,tz,f,k1,k1) as Bundler camera calibrations w/ Rodrigues vector
        and (x,y,z) 3d point initializations.""")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
