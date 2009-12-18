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

	class UrbanFactor : public NonlinearFactor<UrbanConfig> {
	public:
		UrbanFactor();
		virtual ~UrbanFactor();

    /** Vector of errors */
		Vector error_vector(const UrbanConfig& c) const { return zero(0); }

    /** linearize to a GaussianFactor */
    boost::shared_ptr<GaussianFactor> linearize(const UrbanConfig& c) const {
    	boost::shared_ptr<GaussianFactor> factor(new GaussianFactor);
    	return factor;
    }

	};

}

#endif /* URBANFACTOR_H_ */
