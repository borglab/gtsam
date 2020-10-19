"""
  GTSAM Copyright 2010, Georgia Tech Research Corporation,
  Atlanta, Georgia 30332-0415
  All Rights Reserved
  Authors: Frank Dellaert, et al. (see THANKS for the full author list)

  See LICENSE for the license information

  Solve a structure-from-motion problem from a "Bundle Adjustment in the Large" file
  Author: Frank Dellaert (Python: Akshay Krishnan, John Lambert)
"""

import argparse
import logging
import sys

import matplotlib.pyplot as plt
import numpy as np

import gtsam
from gtsam import (
    GeneralSFMFactorCal3Bundler,
    PinholeCameraCal3Bundler,
    PriorFactorPinholeCameraCal3Bundler,
    readBal,
    symbol_shorthand
)

C = symbol_shorthand.C
P = symbol_shorthand.P


logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

def run(args):
    """ Run LM optimization with BAL input data and report resulting error """
    input_file = gtsam.findExampleDataFile(args.input_file)

    # Load the SfM data from file
    scene_data = readBal(input_file)
    logging.info(f"read {scene_data.number_tracks()} tracks on {scene_data.number_cameras()} cameras\n")

    # Create a factor graph
    graph = gtsam.NonlinearFactorGraph()

    # We share *one* noiseModel between all projection factors
    noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0) # one pixel in u and v

    # Add measurements to the factor graph
    j = 0
    for t_idx in range(scene_data.number_tracks()):
        track = scene_data.track(t_idx) # SfmTrack
        # retrieve the SfmMeasurement objects
        for m_idx in range(track.number_measurements()):
            # i represents the camera index, and uv is the 2d measurement
            i, uv = track.measurement(m_idx)
            # note use of shorthand symbols C and P
            graph.add(GeneralSFMFactorCal3Bundler(uv, noise, C(i), P(j)))
        j += 1

    # Add a prior on pose x1. This indirectly specifies where the origin is.
    graph.push_back(
        gtsam.PriorFactorPinholeCameraCal3Bundler(
            C(0), scene_data.camera(0), gtsam.noiseModel.Isotropic.Sigma(9, 0.1)
        )
    )
    # Also add a prior on the position of the first landmark to fix the scale
    graph.push_back(
        gtsam.PriorFactorPoint3(
            P(0), scene_data.track(0).point3(), gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        )
    )

    # Create initial estimate
    initial = gtsam.Values()
    
    i = 0
    # add each PinholeCameraCal3Bundler
    for cam_idx in range(scene_data.number_cameras()):
        camera = scene_data.camera(cam_idx)
        initial.insert(C(i), camera)
        i += 1

    j = 0
    # add each SfmTrack
    for t_idx in range(scene_data.number_tracks()):
        track = scene_data.track(t_idx)  
        initial.insert(P(j), track.point3())
        j += 1

    # Optimize the graph and print results
    try:
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM("ERROR")
        lm = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
        result = lm.optimize()
    except Exception as e:
        logging.exception("LM Optimization failed")
        return
    # Error drops from ~2764.22 to ~0.046
    logging.info(f"final error: {graph.error(result)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-i',
        '--input_file',
        type=str,
        default="dubrovnik-3-7-pre",
        help='Read SFM data from the specified BAL file'
        'The data format is described here: https://grail.cs.washington.edu/projects/bal/.'
        'BAL files contain (nrPoses, nrPoints, nrObservations), followed by (i,j,u,v) tuples, '
        'then (wx,wy,wz,tx,ty,tz,f,k1,k1) as Bundler camera calibrations w/ Rodrigues vector'
        'and (x,y,z) 3d point initializations.'
    )
    run(parser.parse_args())

