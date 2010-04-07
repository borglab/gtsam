/*
 * Simulated2DOrientedPosePrior.h
 *
 * Re-created on Feb 22, 2010 for compatibility with MATLAB
 * Author: Kai Ni
 */

#pragma once

#include "simulated2DOriented.h"
#include "Simulated2DOrientedConfig.h"

namespace gtsam {

	/** Create a prior on a pose Point2 with key 'x1' etc... */
	typedef simulated2DOriented::GenericPosePrior<Simulated2DOrientedConfig, simulated2DOriented::PoseKey> Simulated2DOrientedPosePrior;

}

