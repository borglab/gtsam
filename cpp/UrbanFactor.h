/*
 * UrbanFactor.h
 *
 *  Created on: Dec 17, 2009
 *      Author: Frank Dellaert
 */

#ifndef URBANFACTOR_H_
#define URBANFACTOR_H_

#include "NonlinearFactor.h"
#include "UrbanConfig.h"

namespace gtsam {

	/**
	 * Base class for UrbanMeasurement and UrbanOdometry
	 */
	class UrbanFactor: public NonlinearFactor<UrbanConfig> {
	public:

		UrbanFactor();
		UrbanFactor(const Vector& z, const double sigma);
		virtual ~UrbanFactor();
	};

}

#endif /* URBANFACTOR_H_ */
