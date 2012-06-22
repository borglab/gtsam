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
Sampler::Sampler(const noiseModel::Diagonal::shared_ptr& model, int32_t seed)
	: model_(model), generator_(static_cast<unsigned>(seed))
{
}

/* ************************************************************************* */
Sampler::Sampler(const Vector& sigmas, int32_t seed)
: model_(noiseModel::Diagonal::Sigmas(sigmas, true)), generator_(static_cast<unsigned>(seed))
{
}

/* ************************************************************************* */
Sampler::Sampler(int32_t seed)
: generator_(static_cast<unsigned>(seed))
{
}

/* ************************************************************************* */
Vector Sampler::sampleDiagonal(const Vector& sigmas) {
	size_t d = sigmas.size();
	Vector result(d);
	for (size_t i = 0; i < d; i++) {
		double sigma = sigmas(i);

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

/* ************************************************************************* */
Vector Sampler::sample() {
	assert(model_.get());
	const Vector& sigmas = model_->sigmas();
	return sampleDiagonal(sigmas);
}

/* ************************************************************************* */
Vector Sampler::sampleNewModel(const noiseModel::Diagonal::shared_ptr& model) {
	assert(model.get());
	const Vector& sigmas = model->sigmas();
	return sampleDiagonal(sigmas);
}
/* ************************************************************************* */

} // \namespace gtsam
