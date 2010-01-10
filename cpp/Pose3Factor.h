/**
 *  @file  Pose3Factor.H
 *  @authors Frank Dellaert, Viorela Ila
 **/

#pragma once

#include "Pose3Config.h"
#include "BetweenFactor.h"

namespace gtsam {

	/**
	 * A Factor for 3D pose measurements
	 * This is just a typedef now
	 */
	typedef BetweenFactor<Pose3, Pose3Config> Pose3Factor;

} /// namespace gtsam
