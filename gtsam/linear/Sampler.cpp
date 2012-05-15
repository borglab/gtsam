/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Sampler.cpp
 * @author Alex Cunningham
 */

#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <gtsam/linear/Sampler.h>

namespace gtsam {

/* ************************************************************************* */
Sampler::Sampler(const SharedDiagonal& model, int32_t seed)
	: sigmas_(model->sigmas()), generator_(static_cast<unsigned>(seed))
{
}

/* ************************************************************************* */
Sampler::Sampler(const Vector& sigmas, int32_t seed)
	: sigmas_(sigmas), generator_(static_cast<unsigned>(seed))
{
}

/* ************************************************************************* */
Vector Sampler::sample() {
	size_t d = dim();
	Vector result(d);
	for (size_t i = 0; i < d; i++) {
		double sigma = sigmas_(i);

		// handle constrained case separately
		if (sigma == 0.0) {
			result(i) = 0.0;
		} else {
			typedef boost::normal_distribution<double> Normal;
			Normal dist(0.0, sigma);
			boost::variate_generator<boost::minstd_rand&, Normal> norm(generator_, dist);
			result(i) = norm();
		}
	}
	return result;
}

} // \namespace gtsam
