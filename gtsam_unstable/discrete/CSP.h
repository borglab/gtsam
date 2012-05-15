/*
 * CSP.h
 * @brief Constraint Satisfaction Problem class
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam_unstable/discrete/AllDiff.h>
#include <gtsam_unstable/discrete/SingleValue.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>

namespace gtsam {

	/**
	 * Constraint Satisfaction Problem class
	 * A specialization of a DiscreteFactorGraph.
	 * It knows about CSP-specific constraints and algorithms
	 */
	class CSP: public FactorGraph<Constraint> {
	public:

		/** A map from keys to values */
		typedef std::vector<Index> Indices;
		typedef Assignment<Index> Values;
		typedef boost::shared_ptr<Values> sharedValues;

	public:
		/// Constructor
		CSP() {
		}

		template<class SOURCE>
		void add(const DiscreteKey& j, SOURCE table) {
			DiscreteKeys keys;
			keys.push_back(j);
			push_back(boost::make_shared<DecisionTreeFactor>(keys, table));
		}

		template<class SOURCE>
		void add(const DiscreteKey& j1, const DiscreteKey& j2, SOURCE table) {
			DiscreteKeys keys;
			keys.push_back(j1);
			keys.push_back(j2);
			push_back(boost::make_shared<DecisionTreeFactor>(keys, table));
		}

		/** add shared discreteFactor immediately from arguments */
		template<class SOURCE>
		void add(const DiscreteKeys& keys, SOURCE table) {
			push_back(boost::make_shared<DecisionTreeFactor>(keys, table));
		}

		/// Add a unary constraint, allowing only a single value
		void addSingleValue(const DiscreteKey& dkey, size_t value) {
			boost::shared_ptr<SingleValue> factor(new SingleValue(dkey, value));
			push_back(factor);
		}

		/// Add a binary AllDiff constraint
		void addAllDiff(const DiscreteKey& key1, const DiscreteKey& key2) {
			boost::shared_ptr<BinaryAllDiff> factor(
					new BinaryAllDiff(key1, key2));
			push_back(factor);
		}

		/// Add a general AllDiff constraint
		void addAllDiff(const DiscreteKeys& dkeys) {
			boost::shared_ptr<AllDiff> factor(new AllDiff(dkeys));
			push_back(factor);
		}

		/// Find the best total assignment - can be expensive
		sharedValues optimalAssignment() const;

		/*
		 * Perform loopy belief propagation
		 * True belief propagation would check for each value in domain
		 * whether any satisfying separator assignment can be found.
		 * This corresponds to hyper-arc consistency in CSP speak.
		 * This can be done by creating a mini-factor graph and search.
		 * For a nine-by-nine Sudoku, the search tree will be 8+6+6=20 levels deep.
		 * It will be very expensive to exclude values that way.
		 */
		// void applyBeliefPropagation(size_t nrIterations = 10) const;
		/*
		 * Apply arc-consistency ~ Approximate loopy belief propagation
		 * We need to give the domains to a constraint, and it returns
		 * a domain whose values don't conflict in the arc-consistency way.
		 * TODO: should get cardinality from Indices
		 */
		void runArcConsistency(size_t cardinality, size_t nrIterations = 10,
				bool print = false) const;
	};

} // gtsam

