/**
 *  @file  Pose3Factor.H
 *  @authors Frank Dellaert, Viorela Ila
 **/

#pragma once

#include <map>
#include "Pose3.h"
#include "LieConfig.h"
#include "BetweenFactor.h"

namespace gtsam {

	/**
	 * A config specifically for 3D poses
	 */
	typedef LieConfig<Pose3> Pose3Config;

	/**
	 * A Factor for 3D pose measurements
	 * This is just a typedef now
	 */
	typedef BetweenFactor<Pose3, Pose3Config> Pose3Factor;

} /// namespace gtsam
