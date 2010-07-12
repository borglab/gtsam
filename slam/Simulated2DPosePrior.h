/*
 * Simulated2DPosePrior.h
 *
 * Re-created on Feb 22, 2010 for compatibility with MATLAB
 * Author: Frank Dellaert
 */

#pragma once

#include "simulated2D.h"
#include "Simulated2DConfig.h"

namespace gtsam {

	/** Create a prior on a pose Point2 with key 'x1' etc... */
	typedef simulated2D::GenericPrior<Simulated2DConfig, simulated2D::PoseKey> Simulated2DPosePrior;

}

