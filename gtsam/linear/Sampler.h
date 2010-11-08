/**
 * @file Sampler.h
 * @brief sampling that can be parameterized using a NoiseModel to generate samples from
 * the given distribution
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/linear/SharedDiagonal.h>

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
	/** sigmas from the noise model */
	Vector sigmas_;

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
	Sampler(const SharedDiagonal& model, int32_t seed = 42u);

	/**
	 * Create a sampler for a distribution specified by a vector of sigmas directly
	 *
	 * NOTE: do not use zero as a seed, it will break the generator
	 */
	Sampler(const Vector& sigmas, int32_t seed = 42u);

	/** access functions */
	size_t dim() const { return sigmas_.size(); }
	Vector sigmas() const { return sigmas_; }

	/**
	 * sample from distribution
	 * NOTE: not const due to need to update the underlying generator
	 */
	Vector sample();

};

}
