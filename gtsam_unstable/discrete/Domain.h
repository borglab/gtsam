/*
 * Domain.h
 * @brief Domain restriction constraint
 * @date Feb 13, 2012
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam_unstable/discrete/Constraint.h>
#include <gtsam/discrete/DiscreteKey.h>

namespace gtsam {

	/**
	 * Domain restriction constraint
	 */
	class Domain: public Constraint {

		size_t cardinality_; /// Cardinality
		std::set<size_t> values_; /// allowed values

	public:

		typedef boost::shared_ptr<Domain> shared_ptr;

		// Constructor on Discrete Key initializes an "all-allowed" domain
		Domain(const DiscreteKey& dkey) :
			Constraint(dkey.first), cardinality_(dkey.second) {
			for (size_t v = 0; v < cardinality_; v++)
				values_.insert(v);
		}

		// Constructor on Discrete Key with single allowed value
		// Consider SingleValue constraint
		Domain(const DiscreteKey& dkey, size_t v) :
			Constraint(dkey.first), cardinality_(dkey.second) {
			values_.insert(v);
		}

		/// Constructor
		Domain(const Domain& other) :
			Constraint(other.keys_[0]), values_(other.values_) {
		}

		/// insert a value, non const :-(
		void insert(size_t value) {
			values_.insert(value);
		}

		/// erase a value, non const :-(
		void erase(size_t value) {
			 values_.erase(value);
		}

		size_t nrValues() const {
			return values_.size();
		}

		bool isSingleton() const {
			return nrValues() == 1;
		}

		size_t firstValue() const {
			return *values_.begin();
		}

		// print
		virtual void print(const std::string& s = "",
				const IndexFormatter& formatter = DefaultIndexFormatter) const;

		bool contains(size_t value) const {
			return values_.count(value)>0;
		}

		/// Calculate value
		virtual double operator()(const Values& values) const;

		/// Convert into a decisiontree
		virtual DecisionTreeFactor toDecisionTreeFactor() const;

		/// Multiply into a decisiontree
		virtual DecisionTreeFactor operator*(const DecisionTreeFactor& f) const;

		/*
		 * Ensure Arc-consistency
		 * @param j domain to be checked
		 * @param domains all other domains
		 */
		bool ensureArcConsistency(size_t j, std::vector<Domain>& domains) const;

		/**
		 *  Check for a value in domain that does not occur in any other connected domain.
		 *  If found, we make this a singleton... Called in AllDiff::ensureArcConsistency
		 *  @param keys connected domains through alldiff
		 */
		bool checkAllDiff(const std::vector<Index> keys, std::vector<Domain>& domains);

		/// Partially apply known values
		virtual Constraint::shared_ptr partiallyApply(
				const Values& values) const;

		/// Partially apply known values, domain version
		virtual Constraint::shared_ptr partiallyApply(
				const std::vector<Domain>& domains) const;
	};

} // namespace gtsam
