/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief sampling that can be parameterized using a NoiseModel to generate samples from
 * @file Sampler.h
 * the given distribution
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/linear/NoiseModel.h>

#include <boost/random/linear_congruential.hpp>

namespace gtsam {

/**
 * Sampling structure that keeps internal random number generators for
 * diagonal distributions specified by NoiseModel
 *
 * This is primarily to allow for variable seeds, and does roughly the same
 * thing as sample() in NoiseModel.
 */
class Sampler {
protected:
	/** noiseModel created at generation */
	noiseModel::Diagonal::shared_ptr model_;

	/** generator */
	boost::minstd_rand generator_;

public:
	typedef boost::shared_ptr<Sampler> shared_ptr;

	/**
	 * Create a sampler for the distribution specified by a diagonal NoiseModel
	 * with a manually specified seed
	 *
	 * NOTE: do not use zero as a seed, it will break the generator
	 */
	Sampler(const noiseModel::Diagonal::shared_ptr& model, int32_t seed = 42u);

	/**
	 * Create a sampler for a distribution specified by a vector of sigmas directly
	 *
	 * NOTE: do not use zero as a seed, it will break the generator
	 */
	Sampler(const Vector& sigmas, int32_t seed = 42u);

	/**
	 * Create a sampler without a given noisemodel - pass in to sample
	 *
	 * NOTE: do not use zero as a seed, it will break the generator
	 */
	Sampler(int32_t seed = 42u);

	/** access functions */
	size_t dim() const { assert(model_.get()); return model_->dim(); }
	Vector sigmas() const { assert(model_.get()); return model_->sigmas(); }
	const noiseModel::Diagonal::shared_ptr& model() const { return model_; }

	/**
	 * sample from distribution
	 * NOTE: not const due to need to update the underlying generator
	 */
	Vector sample();

	/**
	 * Sample from noisemodel passed in as an argument,
	 * can be used without having initialized a model for the system.
	 *
	 * NOTE: not const due to need to update the underlying generator
	 */
	Vector sampleNewModel(const noiseModel::Diagonal::shared_ptr& model);

protected:

	/** given sigmas for a diagonal model, returns a sample */
	Vector sampleDiagonal(const Vector& sigmas);

};

} // \namespace gtsam
