/**
 *  @file  Pose3Factor.H
 *  @authors Frank Dellaert, Viorela Ila
 **/

#pragma once

#include <map>
#include "BetweenFactor.h"
#include "Pose3.h"

namespace gtsam {

	/**
	 * A config specifically for 3D poses
	 */
	class Pose3Config: public std::map<std::string, Pose3> {

	public:

		const Pose3& get(std::string key) const {
			const_iterator it = find(key);
			if (it == end()) throw std::invalid_argument("invalid key");
			return it->second;
		}

	};

	/**
	 * A Factor for 3D pose measurements
	 * This is just a typedef now
	 */
	typedef BetweenFactor<Pose3, Pose3Config> Pose3Factor;

} /// namespace gtsam
