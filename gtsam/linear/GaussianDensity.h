/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianDensity.h
 * @brief   A Gaussian Density
 * @author  Frank Dellaert
 * @date    Jan 21, 2012
 */

// \callgraph
#pragma once

#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

	/**
	 * A Gaussian density.
	 *
	 * It is implemented as a GaussianConditional without parents.
	 * The negative log-probability is given by \f$ |Rx - d|^2 \f$
	 * with \f$ \Lambda = \Sigma^{-1} = R^T R \f$ and \f$ \mu = R^{-1} d \f$
	 */
	class GaussianDensity: public GaussianConditional {

	public:

		typedef boost::shared_ptr<GaussianDensity> shared_ptr;

		/// default constructor needed for serialization
		GaussianDensity() :
				GaussianConditional() {
		}

		/// Copy constructor from GaussianConditional
		GaussianDensity(const GaussianConditional& conditional) :
				GaussianConditional(conditional) {
			assert(conditional.nrParents() == 0);
		}

		/// constructor using d, R
		GaussianDensity(Index key, const Vector& d, const Matrix& R,
				const Vector& sigmas) :
				GaussianConditional(key, d, R, sigmas) {
		}

		/// print
		void print(const std::string& = "GaussianDensity",
				const IndexFormatter& formatter =DefaultIndexFormatter) const;

		/// Mean \f$ \mu = R^{-1} d \f$
		Vector mean() const;

		/// Information matrix \f$ \Lambda = R^T R \f$
		Matrix information() const;

		/// Covariance matrix \f$ \Sigma = (R^T R)^{-1} \f$
		Matrix covariance() const;

	};
// GaussianDensity

}// gtsam
