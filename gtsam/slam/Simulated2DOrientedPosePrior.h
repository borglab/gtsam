/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * Simulated2DOrientedPosePrior.h
 *
 * Re-created on Feb 22, 2010 for compatibility with MATLAB
 * Author: Kai Ni
 */

#pragma once

#include <gtsam/slam/simulated2DOriented.h>
#include <gtsam/slam/Simulated2DOrientedValues.h>

namespace gtsam {

	/** Create a prior on a pose Point2 with key 'x1' etc... */
	typedef simulated2DOriented::GenericPosePrior<Simulated2DOrientedValues, simulated2DOriented::PoseKey> Simulated2DOrientedPosePrior;

}

