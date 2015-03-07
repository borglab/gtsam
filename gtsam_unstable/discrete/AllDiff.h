/*
 * AllDiff.h
 * @brief General "all-different" constraint
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam_unstable/discrete/BinaryAllDiff.h>
#include <gtsam/discrete/DiscreteKey.h>

namespace gtsam {

	/**
	 * General AllDiff constraint
	 * Returns 1 if values for all keys are different, 0 otherwise
	 * DiscreteFactors are all awkward in that they have to store two types of keys:
	 * for each variable we have a Index and an Index. In this factor, we
	 * keep the Indices locally, and the Indices are stored in IndexFactor.
	 */
	class AllDiff: public Constraint {

		std::map<Index,size_t> cardinalities_;

		DiscreteKey discreteKey(size_t i) const {
			Index j = keys_[i];
			return DiscreteKey(j,cardinalities_.at(j));
		}

	public:

		/// Constructor
		AllDiff(const DiscreteKeys& dkeys);

		// print
		virtual void print(const std::string& s = "",
				const IndexFormatter& formatter = DefaultIndexFormatter) const;

		/// Calculate value = expensive !
		virtual double operator()(const Values& values) const;

		/// Convert into a decisiontree, can be *very* expensive !
		virtual DecisionTreeFactor toDecisionTreeFactor() const;

		/// Multiply into a decisiontree
		virtual DecisionTreeFactor operator*(const DecisionTreeFactor& f) const;

		/*
		 * Ensure Arc-consistency
		 * Arc-consistency involves creating binaryAllDiff constraints
		 * In which case the combinatorial hyper-arc explosion disappears.
		 * @param j domain to be checked
		 * @param domains all other domains
		 */
		bool ensureArcConsistency(size_t j, std::vector<Domain>& domains) const;

		/// Partially apply known values
		virtual Constraint::shared_ptr partiallyApply(const Values&) const;

		/// Partially apply known values, domain version
		virtual Constraint::shared_ptr partiallyApply(const std::vector<Domain>&) const;
	};

} // namespace gtsam
