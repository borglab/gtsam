/**
 * @file    BayesNet
 * @brief   Bayes network
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "Testable.h"

namespace gtsam {

	class Ordering;

	/**
	 * Bayes network
	 * This is the base class for SymbolicBayesNet, DiscreteBayesNet, and GaussianBayesNet
	 * corresponding to what is used for the "Conditional" template argument:
	 * a SymbolicConditional, ConditionalProbabilityTable, or a GaussianConditional
	 */
	template<class Conditional>
	class BayesNet: public Testable<BayesNet<Conditional> > {

	public:

		/** We store shared pointers to Conditional densities */
		typedef typename boost::shared_ptr<Conditional> sharedConditional;
		typedef typename std::list<sharedConditional> Conditionals;

		typedef typename Conditionals::const_iterator const_iterator;
		typedef typename Conditionals::const_reverse_iterator const_reverse_iterator;

	protected:

		/**
		 *  Conditional densities are stored in reverse topological sort order (i.e., leaves first,
		 *  parents last), which corresponds to the elimination ordering if so obtained,
		 *  and is consistent with the column (block) ordering of an upper triangular matrix.
		 */
		Conditionals conditionals_;

	public:

		/** print */
		void print(const std::string& s = "") const;

		/** check equality */
		bool equals(const BayesNet& other, double tol = 1e-9) const;

		/** push_back: use reverse topological sort (i.e. parents last / elimination order) */
		inline void push_back(const sharedConditional& conditional) {
			conditionals_.push_back(conditional);
		}

		/** push_front: use topological sort (i.e. parents first / reverse elimination order) */
		inline void push_front(const sharedConditional& conditional) {
			conditionals_.push_front(conditional);
		}

		// push_back an entire Bayes net */
		void push_back(const BayesNet<Conditional> bn);

		// push_front an entire Bayes net */
		void push_front(const BayesNet<Conditional> bn);

		/**
		 * pop_front: remove node at the bottom, used in marginalization
		 * For example P(ABC)=P(A|BC)P(B|C)P(C) becomes P(BC)=P(B|C)P(C)
		 */
		inline void pop_front() {conditionals_.pop_front();}

		/** size is the number of nodes */
		inline size_t size() const {
			return conditionals_.size();
		}

		/** return keys in reverse topological sort order, i.e., elimination order */
		Ordering ordering() const;

		/** SLOW O(n) random access to Conditional by key */
		sharedConditional operator[](const std::string& key) const;

		/** return last node in ordering */
		inline sharedConditional back() { return conditionals_.back(); }

		/** return iterators. FD: breaks encapsulation? */
		inline const_iterator const begin() const {return conditionals_.begin();}
		inline const_iterator const end()   const {return conditionals_.end();}
		inline const_reverse_iterator const rbegin() const {return conditionals_.rbegin();}
		inline const_reverse_iterator const rend()   const {return conditionals_.rend();}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(conditionals_);
		}
	}; // BayesNet

} /// namespace gtsam
