#!/usr/bin/env python
from gtsam import *
import numpy_eigen as npe

noisemodel = DiagonalNoiseModel.Sigmas([0.1, 0.1, 0.1])
graph = NonlinearFactorGraph()
initialEstimate = Values()