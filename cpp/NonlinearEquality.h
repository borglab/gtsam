/*
 * @file NonlinearEquality.h
 * @brief Factor to handle enforced equality between factors
 * @author Alex Cunningham
 */

#pragma once

#include <limits>
#include <iostream>
#include "NonlinearFactor.h"

namespace gtsam {

/**
 * An equality factor that forces either one variable to a constant,
 * or a set of variables to be equal to each other.
 * Throws an error at linearization if the constraints are not met.
 */
template<class Config>
class NonlinearEquality : public NonlinearFactor<Config> {
private:

	// node to constrain
	std::string key_;

	// config containing the necessary feasible point
	Config feasible_;

	// dimension of the variable
	size_t dim_;

public:

	/**
	 * Function that compares a value from a config with
	 * another to determine whether a linearization point is
	 * a feasible point.
	 * @param key is the identifier for the key
	 * @param feasible is the value which is constrained
	 * @param input is the config to be tested for feasibility
	 * @return true if the linearization point is feasible
	 */
	bool (*compare_)(const std::string& key, const Config& feasible, const Config& input);

	/** Constructor */
	NonlinearEquality(const std::string& key,
			const Config& feasible,
			size_t dim,
			bool (*compare)(const std::string& key,
							const Config& feasible,
							const Config& input))
	: key_(key), dim_(dim), feasible_(feasible), compare_(compare) {

	}

	void print(const std::string& s = "") const {
		std::cout << "Constraint: " << s << " on [" << key_ << "]\n";
		feasible_.print("Feasible Point");
		std::cout << "Variable Dimension: " << dim_ << std::endl;
	}

	/** Check if two factors are equal */
	bool equals(const Factor<Config>& f, double tol=1e-9) const {
		const NonlinearEquality<Config>* p = dynamic_cast<const NonlinearEquality<Config>*> (&f);
		if (p == NULL) return false;
		if (key_ != p->key_) return false;
		if (!compare_(key_, feasible_, p->feasible_)) return false; // only check the relevant value
		return dim_ == p->dim_;
	}

	/** error function */
	inline Vector error_vector(const Config& c) const {
		if (!compare_(key_, feasible_, c))
			return repeat(dim_, std::numeric_limits<double>::infinity()); // set error to infinity if not equal
		else
			return zero(dim_); // set error to zero if equal
	}

	/** linearize a nonlinear constraint into a linear constraint */
	boost::shared_ptr<GaussianFactor> linearize(const Config& c) const {
		if (!compare_(key_, feasible_, c)) {
			throw std::invalid_argument("Linearization point not feasible for " + key_ + "!");
		} else {
			GaussianFactor::shared_ptr ret(new GaussianFactor(key_, eye(dim_), zero(dim_), 0.0));
			return ret;
		}
	}
};

}

