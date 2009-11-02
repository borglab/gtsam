/**
 * @file    BayesNet
 * @brief   Bayes network
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "Testable.h"

namespace gtsam {

	class Ordering;

	/**
	 * Bayes network
	 * This is the base class for SymbolicBayesNet, DiscreteBayesNet, and GaussianBayesNet
	 * corresponding to what is used for the "Conditional" template argument:
	 * a SymbolicConditional, ConditionalProbabilityTable, or a ConditionalGaussian
	 */
	template<class Conditional>
	class BayesNet: public Testable<BayesNet<Conditional> > {

	public:

		/** We store shared pointers to Conditional densities */
		typedef typename boost::shared_ptr<Conditional> conditional_ptr;
		typedef typename std::vector<conditional_ptr> Conditionals;
		typedef typename Conditionals::const_iterator const_iterator;
		typedef typename Conditionals::const_reverse_iterator const_reverse_iterator;

	protected:

		/**
		 *  Conditional densities are stored in reverse topological sort order (i.e., leaves first,
		 *  parents last), which corresponds to the elimination ordering if so obtained,
		 *  and is consistent with the column (block) ordering of an upper triangular matrix.
		 */
		Conditionals conditionals_;

		/**
		 *   O(log n) random access on keys will provided by a map from keys to vector index.
		 */
		typedef std::map<std::string, int> Indices;
		Indices indices_;

		/** O(log n) lookup from key to node index */
		inline int index(const std::string& key) const {
			Indices::const_iterator it = indices_.find(key); // get node index
			assert( it != indices_.end() );
			return it->second;
		}

	public:

		/** print */
		void print(const std::string& s = "") const;

		/** check equality */
		bool equals(const BayesNet& other, double tol = 1e-9) const;

		/** insert: use reverse topological sort (i.e. parents last / elimination order) */
		void insert(const boost::shared_ptr<Conditional>& conditional);

		/** size is the number of nodes */
		inline size_t size() const {
			return conditionals_.size();
		}

		/** return keys in reverse topological sort order, i.e., elimination order */
		Ordering ordering() const;

		/** O(log n) random access to Conditional by key */
		inline conditional_ptr operator[](const std::string& key) const {
			int i = index(key);
			return conditionals_[i];
		}

		/** return iterators. FD: breaks encapsulation? */
		const_iterator const begin() const {return conditionals_.begin();}
		const_iterator const end()   const {return conditionals_.end();}
		const_reverse_iterator const rbegin() const {return conditionals_.rbegin();}
		const_reverse_iterator const rend()   const {return conditionals_.rend();}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(conditionals_);
			ar & BOOST_SERIALIZATION_NVP(indices_);
		}
	};

} /// namespace gtsam
