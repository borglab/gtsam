/*
 * SingleValue.h
 * @brief domain constraint
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam2/discrete/DiscreteKey.h>
#include <gtsam2/discrete/DiscreteFactor.h>

namespace gtsam {

	/**
	 * SingleValue constraint
	 */
	class SingleValue: public DiscreteFactor {

		/// Number of values
		size_t cardinality_;

		/// allowed value
		size_t value_;

		DiscreteKey discreteKey() const {
			return DiscreteKey(keys_[0],cardinality_);
		}

	public:

		typedef boost::shared_ptr<SingleValue> shared_ptr;

		/// Constructor
		SingleValue(Index key, size_t n, size_t value) :
				DiscreteFactor(key), cardinality_(n), value_(value) {
		}

		/// Constructor
		SingleValue(const DiscreteKey& dkey, size_t value) :
				DiscreteFactor(dkey.first), cardinality_(dkey.second), value_(value) {
		}

		// print
		virtual void print(const std::string& s = "") const;

		/// Calculate value
		virtual double operator()(const Values& values) const;

		/// Convert into a decisiontree
		virtual operator DecisionTreeFactor() const;

		/// Multiply into a decisiontree
		virtual DecisionTreeFactor operator*(const DecisionTreeFactor& f) const;

		/*
		 * Ensure Arc-consistency
		 * @param j domain to be checked
		 * @param domains all other domains
		 */
		bool ensureArcConsistency(size_t j, std::vector<Domain>& domains) const;

		/// Partially apply known values
		virtual DiscreteFactor::shared_ptr partiallyApply(
				const Values& values) const;

		/// Partially apply known values, domain version
		virtual DiscreteFactor::shared_ptr partiallyApply(
				const std::vector<Domain>& domains) const;
	};

} // namespace gtsam
