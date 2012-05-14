/*
 * DiscreteFactor.h
 *
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/discrete/Assignment.h>
#include <gtsam/inference/IndexFactor.h>

namespace gtsam {

	class DecisionTreeFactor;
	class DiscreteConditional;
	class Domain;

	/**
	 * Base class for discrete probabilistic factors
	 * The most general one is the derived DecisionTreeFactor
	 */
	class DiscreteFactor: public IndexFactor {

	public:

		// typedefs needed to play nice with gtsam
		typedef DiscreteFactor This;
		typedef DiscreteConditional ConditionalType;
		typedef boost::shared_ptr<DiscreteFactor> shared_ptr;

		/** A map from keys to values */
		typedef Assignment<Index> Values;
		typedef boost::shared_ptr<Values> sharedValues;

	protected:

		/// Construct n-way factor
		DiscreteFactor(const std::vector<Index>& js) :
				IndexFactor(js) {
		}

		/// Construct unary factor
		DiscreteFactor(Index j) :
				IndexFactor(j) {
		}

		/// Construct binary factor
		DiscreteFactor(Index j1, Index j2) :
				IndexFactor(j1, j2) {
		}

		/// construct from container
		template<class KeyIterator>
		DiscreteFactor(KeyIterator beginKey, KeyIterator endKey) :
				IndexFactor(beginKey, endKey) {
		}

	public:

		/// @name Standard Constructors
		/// @{

		/// Default constructor for I/O
		DiscreteFactor();

		/// Virtual destructor
		virtual ~DiscreteFactor() {}

		/// @}
		/// @name Testable
		/// @{

		// print
		virtual void print(const std::string& s = "DiscreteFactor") const {
			IndexFactor::print(s);
		}

		/// @}
		/// @name Standard Interface
		/// @{

		/// Find value for given assignment of values to variables
		virtual double operator()(const Values&) const = 0;

		/// Multiply in a DecisionTreeFactor and return the result as DecisionTreeFactor
		virtual DecisionTreeFactor operator*(const DecisionTreeFactor&) const = 0;

		virtual operator DecisionTreeFactor() const = 0;

		/*
		 * Ensure Arc-consistency
		 * @param j domain to be checked
		 * @param domains all other domains
		 */
		virtual bool ensureArcConsistency(size_t j, std::vector<Domain>& domains) const = 0;

		/// Partially apply known values
		virtual shared_ptr partiallyApply(const Values&) const = 0;


		/// Partially apply known values, domain version
		virtual shared_ptr partiallyApply(const std::vector<Domain>&) const = 0;
		/// @}
	};
// DiscreteFactor

}// namespace gtsam
