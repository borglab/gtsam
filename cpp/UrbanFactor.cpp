/*
 * UrbanFactor.cpp
 *
 *  Created on: Dec 17, 2009
 *      Author: Frank Dellaert
 */

#include "UrbanFactor.h"

namespace gtsam {

	UrbanFactor::UrbanFactor() {
		// TODO Auto-generated constructor stub
	}

	UrbanFactor::UrbanFactor(const Vector& z, const double sigma) :
		NonlinearFactor<UrbanConfig> (z,sigma) {
		// TODO
	}

	UrbanFactor::~UrbanFactor() {
		// TODO Auto-generated destructor stub
	}

}
