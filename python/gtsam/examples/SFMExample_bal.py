"""
  GTSAM Copyright 2010, Georgia Tech Research Corporation,
  Atlanta, Georgia 30332-0415
  All Rights Reserved
  Authors: Frank Dellaert, et al. (see THANKS for the full author list)

  See LICENSE for the license information

  Solve a structure-from-motion problem from a "Bundle Adjustment in the Large" file
  Author: John Lambert
"""

import argparse
import logging

import gtsam
import matplotlib.pyplot as plt
import numpy as np

from gtsam import symbol_shorthand
from gtsam import readBal

C = symbol_shorthand.C
P = symbol_shorthand.P

import pdb

#include <gtsam/slam/GeneralSFMFactor.h>

# We will be using a projection factor that ties a SFM_Camera to a 3D point.
# An SFM_Camera is defined in datase.h as a camera with unknown Cal3Bundler calibration
# and has a total of 9 free parameters
#typedef GeneralSFMFactor<SfmCamera,Point3> MyFactor;


def run(args):
    """ Run LM optimization with BAL input data and report resulting error """
    # Find default file, but if an argument is given, try loading a file
    if args.input_file:
        input_file = args.input_file
    else:
        input_file = gtsam.findExampleDataFile("dubrovnik-3-7-pre")

    # # Load the SfM data from file
    mydata = readBal(input_file)
    logging.info(f"read {mydata.number_tracks()} tracks on {mydata.number_cameras()} cameras\n")

    # # Create a factor graph
    graph = gtsam.NonlinearFactorGraph()

    # # We share *one* noiseModel between all projection factors
    noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0) # one pixel in u and v

    # Add measurements to the factor graph
    j = 0
    pdb.set_trace()
    for t_idx in range(mydata.number_tracks()):
        track = mydata.track(t_idx) # SfmTrack
        # retrieve the SfmMeasurement objects
        for m_idx in range(track.number_measurements()):
            # i represents the camera index, and uv is the 2d measurement
            i, uv = track.measurement(0) # 
            #graph.emplace_shared<MyFactor>(uv, noise, C(i), P(j)) # note use of shorthand symbols C and P
            #graph.add
        j += 1
    pdb.set_trace()

    # # Add a prior on pose x1. This indirectly specifies where the origin is.
    # # and a prior on the position of the first landmark to fix the scale
    

    graph.push_back(gtsam.PriorFactorVector(
            C(0), mydata.camera(0), gtsam.noiseModel.Isotropic.Sigma(9, 0.1)))
    # # graph.addPrior(P(0), mydata.track(0).p, 
    # equivalent of addPrior
    graph.push_back(gtsam.PriorFactorVector(
            P(0), mydata.track(0)[1], gtsam.noiseModel.Isotropic.Sigma(3, 0.1)))

    # # Create initial estimate
    initial = gtsam.Values()
    
    i = 0
    # add each SfmCamera
    for cam_idx in range(mydata.number_cameras()):
        camera = mydata.camera(0)
        initial.insert(C(i), camera)
        i += 1

    j = 0
    # add each SfmTrack
    for t_idx in range(mydata.number_tracks()):
        track = mydata.track(0)  
        initial.insert(P(j), track.p)
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

    logging.info(f"final error: {graph.error(result)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input_file', type=str, default="",
                        help='Read SFM data from the specified BAL file')
    run(parser.parse_args())




