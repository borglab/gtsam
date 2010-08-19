/*
 * Simulated2DPointPrior.h
 *
 * Re-created on Feb 22, 2010 for compatibility with MATLAB
 * Author: Frank Dellaert
 */

#pragma once

#include <gtsam/slam/simulated2D.h>
#include <gtsam/slam/Simulated2DConfig.h>

namespace gtsam {

	/** Create a prior on a landmark Point2 with key 'l1' etc... */
	typedef simulated2D::GenericPrior<Simulated2DConfig, simulated2D::PointKey> Simulated2DPointPrior;

}

